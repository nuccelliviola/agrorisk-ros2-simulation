"""
Controller Webots del drone, integrato con ROS2 tramite webots_ros2_driver
secondo il pattern "extern controller + driver plugin".

Il movimento è cinematico: a ogni step la posizione del drone viene avvicinata
al target di una quantità fissa, senza simulare la fisica del volo. Questa scelta
permette di concentrarsi sulla logica della missione e sulla cooperazione con il
rover.
"""
import json
import time
from datetime import datetime

from std_msgs.msg import String
import rclpy


def _iso_ms(epoch):
    return datetime.fromtimestamp(epoch).isoformat(timespec='milliseconds')


# DEF del nodo PBRAppearance dell'overlay di allerta di ogni parcella
# in worlds/agro_field.wbt (usato dal Supervisor per l'evidenziazione
# rossa dinamica).
ZONE_FILL_APPEARANCE_DEFS = {
    "ZONA_A": "ZONA_A_ALERT_APPEARANCE",
    "ZONA_B": "ZONA_B_ALERT_APPEARANCE",
    "ZONA_C": "ZONA_C_ALERT_APPEARANCE",
    "ZONA_D": "ZONA_D_ALERT_APPEARANCE",
}
ALERT_COLOR = [0.9, 0.1, 0.1]
ALERT_TRANSPARENCY = 0.5
NEUTRAL_TRANSPARENCY = 1.0

# DEF del nodo PBRAppearance dell'effetto visivo di trattamento di ogni
# parcella. Il rover non e' Supervisor e non puo' quindi mostrare o
# nascondere l'effetto da solo: lo fa il drone, che accede alla scena
# Webots e possiede gia' il meccanismo di evidenziazione delle parcelle.
ZONE_SPRAY_APPEARANCE_DEFS = {
    "ZONA_A": "ZONA_A_SPRAY_APPEARANCE",
    "ZONA_B": "ZONA_B_SPRAY_APPEARANCE",
    "ZONA_C": "ZONA_C_SPRAY_APPEARANCE",
    "ZONA_D": "ZONA_D_SPRAY_APPEARANCE",
}
SPRAY_ACTIVE_TRANSPARENCY = 0.45
SPRAY_HIDDEN_TRANSPARENCY = 1.0

# Colore verde transitorio "appena trattata": acceso subito alla
# ricezione di treatment_done, mantenuto per TREATMENT_GREEN_HOLD_S
# poi spento.
TREATMENT_DONE_COLOR = [0.15, 0.75, 0.2]
TREATMENT_GREEN_HOLD_S = 2.5

# Soglia dell'indice di vegetazione simulato per decidere l'esito della
# verifica. La logica e' analoga all'NDVI: valori piu' bassi indicano
# una possibile condizione di stress della vegetazione, quindi un
# valore sotto soglia corrisponde a un rischio confermato.
STRESS_THRESHOLD = 0.5

# Valore vegetativo simulato associato a ogni parcella: e' il dato che
# il drone "legge" durante la verifica. L'allerta di agro_alert_publisher
# non lo conosce (segnala solo il sospetto, non ha misurato la
# vegetazione), per questo vive qui e non nel messaggio di allerta.
ZONE_STRESS_INDEX = {
    "ZONA_A": 0.70,
    "ZONA_B": 0.72,
    "ZONA_C": 0.34,
    "ZONA_D": 0.68,
}

# Durata della sosta di verifica sulla zona.
VERIFICATION_DURATION_S = 8.0

# Posizione della base (vedi DEF DRONE_BASE e translation di partenza
# di DEF DRONE_1 in worlds/agro_field.wbt).
BASE_X = 0.0
BASE_Y = -14.0
BASE_Z = 0.3

# Rete di sicurezza: senza un timeout, un target malformato o un bug
# futuro lascerebbe la fase "moving"/"returning" a inseguire per sempre
# un target mai raggiunto. Il timeout fa comunque pubblicare un
# "return_to_base" finale del drone. Questo garantisce lo sblocco del
# mission_manager solo nel ramo "rischio non confermato"; nel ramo
# "rischio confermato" il mission_manager attende il return_to_base del
# rover, che non ha un timeout di fase, e quel ramo resta scoperto.
PHASE_TIMEOUT_S = 60.0


class DroneControllerWebots:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        # rclpy.init() va chiamato prima di create_node(). La guardia
        # con rclpy.ok() evita un doppio init nello stesso processo,
        # possibile con respawn=True nel launch file.
        if not rclpy.ok():
            rclpy.init(args=None)
        self.__node = rclpy.create_node('drone_controller_webots')

        self.__target = None
        self.__moving = False
        self.__verifying = False
        self.__returning = False
        # Esito dell'ultima verifica: nel ramo "rischio confermato" il
        # rientro del drone NON chiude la missione (la chiude il rover),
        # quindi cambia solo la nota dell'evento return_to_base.
        self.__risk_confirmed = False
        self.__flight_start_time = None
        self.__verification_start_time = None
        self.__return_start_time = None
        self.__speed = 0.03

        self.__node.create_subscription(
            String, '/agro/mission_cmd', self._on_mission_cmd, 10)
        self.__node.create_subscription(
            String, '/agro/treatment_done', self._on_treatment_done, 10)
        # Serve solo a sapere quando il rover avvia il trattamento, per
        # accendere il getto. Gli eventi pubblicati da questo stesso nodo
        # (flight_to_zone, verification_*...) tornano indietro ma vengono
        # ignorati dal filtro su event_type.
        self.__node.create_subscription(
            String, '/agro/events', self._on_agro_event, 10)
        self.__event_pub = self.__node.create_publisher(
            String, '/agro/events', 10)
        # Il drone pubblica qui l'esito positivo della verifica: il rover
        # usera' questo topic per avviare la missione di intervento.
        self.__risk_confirmed_pub = self.__node.create_publisher(
            String, '/agro/risk_confirmed', 10)

        self.__translation_field = self.__robot.getSelf().getField(
            'translation')

        # Campi 'baseColor'/'transparency' del fill di ogni parcella,
        # per l'evidenziazione rossa dinamica via Supervisor.
        self.__zone_fields = {}
        for zone_id, def_name in ZONE_FILL_APPEARANCE_DEFS.items():
            node = self.__robot.getFromDef(def_name)
            if node is not None:
                self.__zone_fields[zone_id] = {
                    "baseColor": node.getField("baseColor"),
                    "transparency": node.getField("transparency"),
                }

        # Campo 'transparency' dell'effetto di trattamento di ogni
        # parcella (il colore resta quello fisso definito nel .wbt).
        self.__spray_fields = {}
        for zone_id, def_name in ZONE_SPRAY_APPEARANCE_DEFS.items():
            node = self.__robot.getFromDef(def_name)
            if node is not None:
                self.__spray_fields[zone_id] = node.getField("transparency")

        # Timer one-shot per il mantenimento del verde transitorio: uno
        # per zona, cosi' una nuova missione su un'altra zona non
        # interferisce con un hold ancora in corso su quella precedente.
        self.__green_hold_timers = {}

    def _on_mission_cmd(self, msg):
        cmd = json.loads(msg.data)
        self.__target = cmd
        self.__moving = True
        self.__risk_confirmed = False
        self.__flight_start_time = time.time()
        # Cancella eventuali timer del verde ancora attivi sulla stessa
        # zona, evitando che spengano l'evidenziazione rossa appena accesa.
        self._cancel_green_hold_timer(cmd["zone_id"])
        self._highlight_zone(cmd["zone_id"], active=True)

    def _on_agro_event(self, msg):
        event = json.loads(msg.data)
        if event.get("event_type") != "treatment_started":
            return
        zone_id = event.get("zone_id")
        if zone_id is None:
            return
        self._show_spray(zone_id, active=True)

    def _on_treatment_done(self, msg):
        payload = json.loads(msg.data)
        zone_id = payload.get("zone_id")
        if zone_id is None:
            self.__node.get_logger().warn(
                "treatment_done ricevuto senza zone_id, ignorato.")
            return
        self._show_spray(zone_id, active=False)
        self._set_zone_color(zone_id, TREATMENT_DONE_COLOR, ALERT_TRANSPARENCY)
        self.__node.get_logger().info(
            f"Parcella {zone_id} evidenziata: VERDE (trattamento completato)")
        self._start_green_hold_timer(zone_id)

    def _show_spray(self, zone_id, active):
        field = self.__spray_fields.get(zone_id)
        if field is None:
            self.__node.get_logger().warn(
                f"Nessun DEF getto trovato per {zone_id}, effetto saltato.")
            return
        field.setSFFloat(
            SPRAY_ACTIVE_TRANSPARENCY if active else SPRAY_HIDDEN_TRANSPARENCY)

    def _set_zone_color(self, zone_id, color, transparency):
        fields = self.__zone_fields.get(zone_id)
        if fields is None:
            self.__node.get_logger().warn(
                f"Nessun DEF fill trovato per {zone_id}, colore saltato.")
            return
        fields["baseColor"].setSFColor(color)
        fields["transparency"].setSFFloat(transparency)

    def _start_green_hold_timer(self, zone_id):
        self._cancel_green_hold_timer(zone_id)
        timer = self.__node.create_timer(
            TREATMENT_GREEN_HOLD_S,
            lambda: self._on_green_hold_elapsed(zone_id))
        self.__green_hold_timers[zone_id] = timer

    def _cancel_green_hold_timer(self, zone_id):
        timer = self.__green_hold_timers.pop(zone_id, None)
        if timer is not None:
            timer.cancel()
            self.__node.destroy_timer(timer)

    def _on_green_hold_elapsed(self, zone_id):
        self._cancel_green_hold_timer(zone_id)
        self._highlight_zone(zone_id, active=False)
        self.__node.get_logger().info(
            f"Parcella {zone_id} evidenziata: neutro (fine hold verde)")

    def _highlight_zone(self, zone_id, active):
        fields = self.__zone_fields.get(zone_id)
        if fields is None:
            self.__node.get_logger().warn(
                f"Nessun DEF fill trovato per {zone_id}, "
                "evidenziazione saltata.")
            return
        if active:
            fields["baseColor"].setSFColor(ALERT_COLOR)
            fields["transparency"].setSFFloat(ALERT_TRANSPARENCY)
        else:
            fields["transparency"].setSFFloat(NEUTRAL_TRANSPARENCY)
        self.__node.get_logger().info(
            f"Parcella {zone_id} evidenziata: "
            f"{'ROSSO' if active else 'neutro'}")

    def _get_position(self):
        return tuple(self.__translation_field.getSFVec3f())

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        if self.__target is None:
            return

        if self.__moving:
            self._move_towards_target()
        elif self.__verifying:
            self._check_verification_done()
        elif self.__returning:
            self._move_towards_base()

    def _advance_towards(self, tx, ty, tz):
        """Avvicina il drone di un passo verso (tx,ty,tz). Ritorna True
        quando la distanza e' sotto soglia (target raggiunto)."""
        x, y, z = self._get_position()
        dx, dy, dz = tx - x, ty - y, tz - z
        dist = (dx**2 + dy**2 + dz**2) ** 0.5

        if dist < 0.05:
            return True

        step_x = x + (dx / dist) * self.__speed
        step_y = y + (dy / dist) * self.__speed
        step_z = z + (dz / dist) * self.__speed
        self.__translation_field.setSFVec3f([step_x, step_y, step_z])
        return False

    def _zone_outcome(self, zone_id):
        """Fallback difensivo (mai atteso: ZONE_STRESS_INDEX copre tutte
        le zone): se un valore mancasse, tratta la zona come sana, per
        non confermare un rischio senza una misura."""
        indice_stress = ZONE_STRESS_INDEX.get(zone_id, 1.0)
        return indice_stress, indice_stress < STRESS_THRESHOLD

    def _move_towards_target(self):
        if self._phase_timed_out(self.__flight_start_time):
            self._abort_mission("timeout in volo verso la zona")
            return

        tx = self.__target["target_x"]
        ty = self.__target["target_y"]
        tz = self.__target["target_z"]

        if self._advance_towards(tx, ty, tz):
            self.__moving = False
            self.__verifying = True
            arrival_time = time.time()
            self.__verification_start_time = arrival_time
            self._publish_event(self.__target, "flight_to_zone", tx, ty, tz,
                                 "Drone arrivato sulla zona",
                                 self.__flight_start_time, arrival_time)
            self._publish_event(self.__target, "verification_started", tx, ty, tz,
                                 "Verifica avviata: lettura indice di stress vegetativo",
                                 arrival_time, arrival_time)

    def _check_verification_done(self):
        if time.time() - self.__verification_start_time > VERIFICATION_DURATION_S:
            x, y, z = self._get_position()
            end_time = time.time()
            self.__verifying = False

            zone_id = self.__target["zone_id"]
            indice_stress, confirmed = self._zone_outcome(zone_id)
            self.__risk_confirmed = confirmed

            note = (f"Rischio confermato (indice_stress={indice_stress})"
                    if confirmed else
                    f"Rischio non confermato (indice_stress={indice_stress})")
            self._publish_event(self.__target, "verification_result", x, y, z,
                                 note, self.__verification_start_time, end_time,
                                 indice_stress=indice_stress)

            if confirmed:
                self._publish_risk_confirmed(self.__target, indice_stress)
            else:
                self._highlight_zone(self.__target["zone_id"], active=False)

            self.__returning = True
            self.__return_start_time = end_time

    def _publish_risk_confirmed(self, cmd, indice_stress):
        # Il contesto di missione (alert_id, mission_id, action) viene
        # propagato al rover: senza, gli eventi del rover non sarebbero
        # associabili alla missione, e il mission_manager non potrebbe
        # riconoscere per quale missione tenere aperta l'attesa del rover.
        payload = {
            "zone_id": cmd["zone_id"],
            "indice_stress": indice_stress,
            "alert_id": cmd["alert_id"],
            "mission_id": cmd["mission_id"],
            "action": cmd["action"],
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.__risk_confirmed_pub.publish(msg)
        self.__node.get_logger().info(f"Rischio confermato pubblicato: {msg.data}")

    def _move_towards_base(self):
        if self._phase_timed_out(self.__return_start_time):
            self._abort_mission("timeout nel rientro alla base")
            return

        if self._advance_towards(BASE_X, BASE_Y, BASE_Z):
            x, y, z = self._get_position()
            end_time = time.time()
            self.__returning = False
            note = ("Drone tornato alla base, in attesa del completamento "
                    "del rover" if self.__risk_confirmed else
                    "Drone tornato alla base, missione conclusa")
            self._publish_event(self.__target, "return_to_base", x, y, z,
                                 note,
                                 self.__return_start_time, end_time)
            self.__target = None

    def _phase_timed_out(self, phase_start_time):
        return (time.time() - phase_start_time) > PHASE_TIMEOUT_S

    def _abort_mission(self, reason):
        """Pubblica un mission_error e un return_to_base del drone
        (source="drone") invece di lasciare la fase appesa. Nel ramo
        "rischio non confermato" questo chiude comunque la missione lato
        mission_manager; nel ramo "rischio confermato", se il
        mission_manager sta gia' attendendo il rover, ignora questo
        return_to_base e la missione resta aperta (limite noto)."""
        x, y, z = self._get_position()
        now = time.time()
        t_start = (self.__return_start_time if self.__returning
                   else self.__flight_start_time)

        self.__moving = False
        self.__verifying = False
        self.__returning = False

        self._publish_event(self.__target, "mission_error", x, y, z,
                             reason, t_start, now)
        self._publish_event(self.__target, "return_to_base", x, y, z,
                             f"Missione interrotta ({reason})", t_start, now)
        self.__target = None

    def _publish_event(self, cmd, event_type, x, y, z, note, t_start, t_end,
                        indice_stress=None):
        event = {
            "timestamp_start": _iso_ms(t_start),
            "timestamp_end": _iso_ms(t_end),
            "duration_seconds": round(t_end - t_start, 3),
            "event_type": event_type,
            "alert_id": cmd["alert_id"],
            "mission_id": cmd["mission_id"],
            "zone_id": cmd["zone_id"],
            "drone_id": cmd["drone_id"],
            "action": cmd["action"],
            "source": "drone",
            "x": x, "y": y, "z": z,
            "note": note,
        }
        if indice_stress is not None:
            event["indice_stress"] = indice_stress
        msg = String()
        msg.data = json.dumps(event)
        self.__event_pub.publish(msg)