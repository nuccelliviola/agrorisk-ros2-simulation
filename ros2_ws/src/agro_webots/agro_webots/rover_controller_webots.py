"""
Dopo l'arrivo sulla zona, il rover sosta per un
trattamento simulato, pubblica l'esito su /agro/treatment_done (letto
dal drone per spegnere l'evidenziazione: il rover non e' Supervisor), poi 
rientra alla base riusando le STESSE funzioni
di navigazione "orienta poi avanza" usate per l'andata.

Le coordinate delle zone sono le STESSE usate da mission_manager per
instradare il drone (ZONE_POSITIONS): importate direttamente da li',
non duplicate qui, per evitare che le due tabelle possano divergere.
"""
import json
import math
import time
from datetime import datetime

from std_msgs.msg import String
import rclpy

from agro_mission.mission_manager import ZONE_POSITIONS

#funzione per il timestamp
def _iso_ms(epoch):
    return datetime.fromtimestamp(epoch).isoformat(timespec='milliseconds')


WHEEL_MOTOR_NAMES = [
    "wheel_fl_motor", #front left
    "wheel_fr_motor", #front right
    "wheel_rl_motor", #rear left
    "wheel_rr_motor", #rear right
]
#divisione ruote appartenenti al lato sinistro ed al lato destro. Essenziale per il rover"skid-steer".
LEFT_WHEELS = (0, 2)   # wheel_fl_motor, wheel_rl_motor
RIGHT_WHEELS = (1, 3)  # wheel_fr_motor, wheel_rr_motor

FORWARD_VELOCITY = 8.0  # Verificato in un banco di prova rettilineo dedicato: stabile fino a 8.0,
                        # instabile da 12.0 in su (il rover si impenna/ribalta).
TURN_VELOCITY = 4.0     # Per tenere il movimento controllato e stabile.

ANGLE_TOLERANCE_RAD = math.radians(3.0) # Il rover considera di essere "abbastanza orientato"
                                        # verso il target se l'errore angolare è entro 3 gradi.

ARRIVAL_DISTANCE_M = 1.5 # Il rover considera raggiunto il target quando si trova entro
                         # 1.5 metri dalla zona. 

# Durata della sosta di trattamento sulla zona.
TREATMENT_DURATION_S = 8.0

# Posizione di partenza del rover.
ROVER_BASE_X = 2.5
ROVER_BASE_Y = -14.0


class RoverControllerWebots:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot # Salva il riferimento al robot Webots.

        if not rclpy.ok(): # Controllo che ROS2 non sia già stato inizializzato.
            rclpy.init(args=None) 
        self.__node = rclpy.create_node('rover_controller_webots')  # Creo il nodo ROS2 interno al controller Webots.
                                                                    # Quindi il controller è dentro Webots, ma comunica 
                                                                    # con il resto del sistema come nodo ROS2.
        self.__motors = [
            self.__robot.getDevice(name) for name in WHEEL_MOTOR_NAMES] # recupero dei motori
        for motor in self.__motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0) 

        timestep = int(self.__robot.getBasicTimeStep()) # prendo il timestamp della simulazione Webots
        self.__gps = self.__robot.getDevice('gps') # recupero il gps del rover. Mi serve per conoscerne la posizione nel mondo.
        self.__gps.enable(timestep)
        self.__imu = self.__robot.getDevice('inertial_unit') # recupero l'unità inerziale. Mi serve per conoscere l'orientamento
                                                             # del rover, in particolare lo yaw (l'angolo di rotazione sul piano).
        self.__imu.enable(timestep)

        self.__target = None      # (tx, ty) della zona o della base
        self.__rotating = False
        self.__advancing = False
        self.__treating = False
        self.__return_trip = False  # False = verso la zona, True = verso la base
        self.__treatment_start_time = None
        self.__zone_id = None
        self.__indice_stress = None
        # Contesto di missione propagato dal drone via /agro/risk_confirmed:
        # serve a marcare gli eventi del rover con la missione che li ha
        # generati (il rover non riceve /agro/mission_cmd).
        self.__alert_id = None
        self.__mission_id = None

        # Coda FIFO delle richieste (risk_confirmed) arrivate a rover
        # occupato: [payload, payload, ...]. Si svuota di un elemento
        # ogni volta che il rover si libera (rientro alla base
        # completato), stessa filosofia di _alert_queue in
        # mission_manager.py.
        self.__request_queue = []

        self.__node.create_subscription(    # il rover si sottoscrive a /agro/risk_confirmed
            String, '/agro/risk_confirmed', self._on_risk_confirmed, 10) 
        self.__event_pub = self.__node.create_publisher(
            String, '/agro/events', 10)     # il rover pubblica eventi generali sul topic /agro/events, salvati poi dal mission_logger
        self.__treatment_done_pub = self.__node.create_publisher(
            String, '/agro/treatment_done', 10) # pubblica su /agro/treatment_done quando il trattamento è completato.

    def _on_risk_confirmed(self, msg):
        payload = json.loads(msg.data)
        zone_id = payload["zone_id"]
        if ZONE_POSITIONS.get(zone_id) is None:
            self.__node.get_logger().warn(
                f"Zona sconosciuta in risk_confirmed: {zone_id}, ignorato.")
            return

        if self.__target is not None: # rover occupato (zona/trattamento/rientro in corso)
            if self._is_duplicate_zone(zone_id):
                self.__node.get_logger().info(
                    f"Zona {zone_id} gia' in trattamento o in coda, "
                    "richiesta non riaccodata.")
                return
            self._enqueue_request(payload)
            return

        self._start_mission(payload)

    def _is_duplicate_zone(self, zone_id):
        """Una zona e' considerata 'gia' gestita' se e' quella su cui
        il rover e' attualmente occupato (zona/trattamento/rientro) o
        se ha gia' una richiesta in coda."""
        if self.__zone_id == zone_id:
            return True
        return any(item["zone_id"] == zone_id
                   for item in self.__request_queue)

    def _enqueue_request(self, payload):
        self.__request_queue.append(payload)
        self.__node.get_logger().info(
            f"Richiesta accodata: zona={payload['zone_id']}, "
            f"lunghezza coda={len(self.__request_queue)}")
        now = time.time()
        self._publish_event(
            "request_queued", 0.0, 0.0, 0.0,
            (f"Richiesta accodata (posizione {len(self.__request_queue)} "
             f"in coda), rover occupato su {self.__zone_id}"),
            now, now,
            indice_stress=payload.get("indice_stress"),
            zone_id=payload["zone_id"],
            alert_id=payload.get("alert_id"),
            mission_id=payload.get("mission_id"))

    def _dequeue_next_request(self):
        if not self.__request_queue:
            return

        payload = self.__request_queue.pop(0)
        self.__node.get_logger().info(
            f"Richiesta sfilata dalla coda: zona={payload['zone_id']}, "
            f"lunghezza coda={len(self.__request_queue)}")
        self._start_mission(payload)

    def _start_mission(self, payload):
        zone_id = payload["zone_id"]
        target = ZONE_POSITIONS[zone_id]

        self.__zone_id = zone_id
        self.__indice_stress = payload.get("indice_stress")
        self.__alert_id = payload.get("alert_id")
        self.__mission_id = payload.get("mission_id")
        self.__target = target
        self.__return_trip = False
        self.__rotating = True # avvia la prima fase : orientarsi verso il target.
        self.__node.get_logger().info(
            f"Rover: navigazione avviata verso {zone_id} {target}")

    def _get_pose(self):
        x, y, _ = self.__gps.getValues() # prende la posizione dal GPS. Mi servono solo x e y, quindi la z la metto in _.
        yaw = self.__imu.getRollPitchYaw()[2] # L'unità inerziale restituisce : roll, pitch, yaw. Io prendo solo yaw. ([2])
        return x, y, yaw

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0) # permette al nodo ROS2 interno di ricevere messaggi.

        if self.__target is None: 
            return                  # se non ha un target non deve fare nulla.

        if self.__rotating:
            self._rotate_towards_target()
        elif self.__advancing:
            self._advance_towards_target()
        elif self.__treating:
            self._check_treatment_done()

    # Metodo per impostare la velocità delle ruote sinistre e destre.
    def _set_wheel_velocities(self, left, right): 
        for i in LEFT_WHEELS:
            self.__motors[i].setVelocity(left) # per ogni ruota sinistra, imposta velocità left.
        for i in RIGHT_WHEELS:
            self.__motors[i].setVelocity(right) # per ogni ruota destra, imposta velocità right.

    # Metodo per far orientare il rover verso il target prima di farlo avanzare.
    def _rotate_towards_target(self):
        x, y, yaw = self._get_pose() # posizione e orientamento attuale del rover
        tx, ty = self.__target      # coordinate del target.
        desired_yaw = math.atan2(ty - y, tx - x) # calcola l'angolo verso il target.
        diff = math.atan2(math.sin(desired_yaw - yaw), math.cos(desired_yaw - yaw)) # calcola la differenza tra direzione desiderata 
                                                                                    # e orientamento attuale.

        if abs(diff) <= ANGLE_TOLERANCE_RAD: # il rover è orientato
            self.__rotating = False
            self.__advancing = True
            self._set_wheel_velocities(FORWARD_VELOCITY, FORWARD_VELOCITY)
            return

        # Se il rover non è ancora orientato sceglie da che lato girare.
        # diff > 0: bersaglio a sinistra (yaw deve crescere) -> ruote
        # destre avanti, sinistre indietro (rotazione antioraria sul
        # posto). 
        turn = TURN_VELOCITY if diff > 0 else -TURN_VELOCITY
        self._set_wheel_velocities(-turn, turn)

    def _advance_towards_target(self):
        x, y, _ = self._get_pose()
        tx, ty = self.__target
        dist = math.hypot(tx - x, ty - y) # calcola la distanza tra rover e target (distanza euclidea)

        if dist <= ARRIVAL_DISTANCE_M:
            self.__advancing = False # target raggiunto, il rover smette di avanzare.
            self._set_wheel_velocities(0.0, 0.0)

            if self.__return_trip:
                now = time.time()
                self._publish_event("return_to_base", x, y, 0.0,
                                     "Rover rientrato alla base dopo il trattamento",
                                     now, now)
                self.__node.get_logger().info("Rover: rientrato alla base, fermo.")
                self.__target = None
                self._dequeue_next_request()
            else:
                self.__treating = True
                self.__treatment_start_time = time.time()
                self._publish_event("treatment_started", x, y, 0.0,
                                     f"Trattamento avviato su {self.__zone_id}",
                                     self.__treatment_start_time,
                                     self.__treatment_start_time)
                self.__node.get_logger().info(
                    f"Rover: arrivato su {self.__zone_id}, trattamento avviato.")

    def _check_treatment_done(self):
        if time.time() - self.__treatment_start_time > TREATMENT_DURATION_S:
            x, y, _ = self._get_pose()
            end_time = time.time()
            self.__treating = False

            self._publish_event("treatment_result", x, y, 0.0,
                                 f"Trattamento completato su {self.__zone_id}",
                                 self.__treatment_start_time, end_time,
                                 indice_stress=self.__indice_stress)
            self._publish_treatment_done()
            self._start_return_to_base()

    def _publish_treatment_done(self):
        payload = {"zone_id": self.__zone_id}
        msg = String()
        msg.data = json.dumps(payload)
        self.__treatment_done_pub.publish(msg)
        self.__node.get_logger().info(
            f"Trattamento completato pubblicato: {msg.data}")

    def _start_return_to_base(self):
        self.__target = (ROVER_BASE_X, ROVER_BASE_Y)
        self.__return_trip = True
        self.__rotating = True
        self.__node.get_logger().info("Rover: rientro alla base avviato.")

    def _publish_event(self, event_type, x, y, z, note, t_start, t_end,
                        indice_stress=None, zone_id=None,
                        alert_id=None, mission_id=None):
        event = {
            "timestamp_start": _iso_ms(t_start),
            "timestamp_end": _iso_ms(t_end),
            "duration_seconds": round(t_end - t_start, 3),
            "event_type": event_type,
            "alert_id": alert_id if alert_id is not None else self.__alert_id,
            "mission_id": (mission_id if mission_id is not None
                           else self.__mission_id),
            "zone_id": zone_id if zone_id is not None else self.__zone_id,
            # drone_id resta vuoto: questi eventi non li produce il drone.
            "source": "rover",
            "x": x, "y": y, "z": z,
            "note": note,
        }
        if indice_stress is not None:
            event["indice_stress"] = indice_stress
        msg = String()
        msg.data = json.dumps(event)
        self.__event_pub.publish(msg)
