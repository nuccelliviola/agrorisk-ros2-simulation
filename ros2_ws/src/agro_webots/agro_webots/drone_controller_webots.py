"""
Controller Webots del drone integrato con ROS2 tramite webots_ros2_driver.

Il movimento è cinematico: la posizione viene aggiornata direttamente verso
il target senza modellare la dinamica fisica del volo.
"""

import json
import time
from datetime import datetime

from std_msgs.msg import String
import rclpy


def _iso_ms(epoch):
    return datetime.fromtimestamp(epoch).isoformat(timespec='milliseconds')


# DEF degli overlay utilizzati dal Supervisor per evidenziare le parcelle.
ZONE_FILL_APPEARANCE_DEFS = {
    "ZONA_A": "ZONA_A_ALERT_APPEARANCE",
    "ZONA_B": "ZONA_B_ALERT_APPEARANCE",
    "ZONA_C": "ZONA_C_ALERT_APPEARANCE",
    "ZONA_D": "ZONA_D_ALERT_APPEARANCE",
}
ALERT_COLOR = [0.9, 0.1, 0.1]
ALERT_TRANSPARENCY = 0.5
NEUTRAL_TRANSPARENCY = 1.0

# DEF degli effetti grafici associati al trattamento.
# La scena viene modificata dal drone, configurato come Supervisor.
ZONE_SPRAY_APPEARANCE_DEFS = {
    "ZONA_A": "ZONA_A_SPRAY_APPEARANCE",
    "ZONA_B": "ZONA_B_SPRAY_APPEARANCE",
    "ZONA_C": "ZONA_C_SPRAY_APPEARANCE",
    "ZONA_D": "ZONA_D_SPRAY_APPEARANCE",
}
SPRAY_ACTIVE_TRANSPARENCY = 0.45
SPRAY_HIDDEN_TRANSPARENCY = 1.0

# Evidenziazione temporanea della parcella al termine del trattamento.
TREATMENT_DONE_COLOR = [0.15, 0.75, 0.2]
TREATMENT_GREEN_HOLD_S = 2.5

# Soglia dell'indice vegetativo simulato: valori inferiori indicano
# una possibile condizione di stress e confermano operativamente il rischio.
STRESS_THRESHOLD = 0.5

# Valore vegetativo sintetico associato a ciascuna parcella.
# È separato dall'allerta AgroRisk perché rappresenta l'esito della
# successiva verifica aerea, non un'informazione già nota al publisher.
ZONE_STRESS_INDEX = {
    "ZONA_A": 0.70,
    "ZONA_B": 0.72,
    "ZONA_C": 0.34,
    "ZONA_D": 0.68,
}

VERIFICATION_DURATION_S = 8.0


BASE_X = 0.0
BASE_Y = -14.0
BASE_Z = 0.3

# Timeout difensivo per evitare che una fase di movimento rimanga
# indefinitamente attiva in caso di target non raggiungibile. E' basato
# su tempo reale (non simulato): un margine ampio evita falsi allarmi
# quando Webots esegue più lentamente del tempo reale.
# Il ramo cooperativo rimane comunque dipendente dal rientro del rover.
PHASE_TIMEOUT_S = 180.0


class DroneControllerWebots:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        if not rclpy.ok():
            rclpy.init(args=None)
        self.__node = rclpy.create_node('drone_controller_webots')

        self.__target = None
        self.__moving = False
        self.__verifying = False
        self.__returning = False
        # Memorizza l'esito della verifica per distinguere i due rami della missione.
        self.__risk_confirmed = False
        self.__flight_start_time = None
        self.__verification_start_time = None
        self.__return_start_time = None
        self.__speed = 0.03
        self.__node.create_subscription(
            String, '/agro/mission_cmd', self._on_mission_cmd, 10)
        self.__node.create_subscription(
            String, '/agro/treatment_done', self._on_treatment_done, 10)
        # Utilizzato per visualizzare l'avvio del trattamento eseguito dal rover.
        self.__node.create_subscription(
            String, '/agro/events', self._on_agro_event, 10)
        self.__event_pub = self.__node.create_publisher(
            String, '/agro/events', 10)
        # Pubblica la conferma utilizzata dal rover e dal mission_manager.
        self.__risk_confirmed_pub = self.__node.create_publisher(
            String, '/agro/risk_confirmed', 10)

        self.__translation_field = self.__robot.getSelf().getField(
            'translation')

        # Campi grafici modificabili dal Supervisor per ciascuna parcella.
        self.__zone_fields = {}
        for zone_id, def_name in ZONE_FILL_APPEARANCE_DEFS.items():
            node = self.__robot.getFromDef(def_name)
            if node is not None:
                self.__zone_fields[zone_id] = {
                    "baseColor": node.getField("baseColor"),
                    "transparency": node.getField("transparency"),
                }

        # Trasparenza dell'effetto grafico di trattamento.
        self.__spray_fields = {}
        for zone_id, def_name in ZONE_SPRAY_APPEARANCE_DEFS.items():
            node = self.__robot.getFromDef(def_name)
            if node is not None:
                self.__spray_fields[zone_id] = node.getField("transparency")

        # Timer indipendente per l'evidenziazione temporanea di ciascuna zona.
        self.__green_hold_timers = {}

    def _on_mission_cmd(self, msg):
        cmd = json.loads(msg.data)
        self.__target = cmd
        self.__moving = True
        self.__risk_confirmed = False
        self.__flight_start_time = time.time()
        # Evita interferenze con una precedente evidenziazione temporanea.
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
        """Fallback difensivo: se la zona non dispone di un indice configurato,
        il rischio non viene confermato in assenza di una misura disponibile."""
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
        # Propaga al rover il contesto dell'allerta e della missione corrente.
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
        """
        Interrompe la fase corrente e pubblica un evento mission_error.

        Nel ramo non confermato il successivo return_to_base consente al
        mission_manager di chiudere la missione. Se il rover è già atteso,
        il completamento resta invece dipendente dal suo rientro.
        """
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