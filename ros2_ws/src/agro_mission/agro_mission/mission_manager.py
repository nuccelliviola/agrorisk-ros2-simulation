"""
Riceve le allerte da /agro/alert, decide se avviare una missione
(una sola missione attiva alla volta), assegna un drone e pubblica:
- il comando di missione su /agro/mission_cmd (per il drone_controller)
- gli eventi di missione su /agro/events (per il mission_logger)

Ascolta anche /agro/events per sapere quando una missione e' conclusa.
Il segnale di fine missione e' l'evento "return_to_base", ma quale
return_to_base conti dipende dal ramo:
- rischio non confermato: chiude la missione il return_to_base del drone
  (source="drone");
- rischio confermato: il return_to_base del drone viene ignorato e si
  attende quello del rover (source="rover"). Il mission_manager riconosce
  questo ramo perche' e' sottoscritto a /agro/risk_confirmed (flag
  interno _waiting_for_rover).
Alla chiusura, il mission_manager torna disponibile per la prossima
allerta e pubblica la riga riepilogativa "mission" (durata totale, da
alert_received al return_to_base che ha concluso la missione).


"""
import json
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _iso_ms(epoch=None):
    return datetime.fromtimestamp(
        epoch if epoch is not None else time.time()
    ).isoformat(timespec='milliseconds')


# Posizione (x, y) associata a ciascuna zona nel mondo Webots.
# Deve corrispondere ai centri delle DEF ZONA_*_PARCEL in
# worlds/agro_field.wbt (griglia 2x2 di parcelle 7.5x7.5m centrate a
# +-4.75m).
ZONE_POSITIONS = {
    "ZONA_A": (-4.75, 4.75),
    "ZONA_B": (4.75, 4.75),
    "ZONA_C": (-4.75, -4.75),
    "ZONA_D": (4.75, -4.75),
}


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        self.alert_sub = self.create_subscription(
            String, '/agro/alert', self.on_alert, 10)
        self.event_sub = self.create_subscription(
            String, '/agro/events', self.on_event, 10)
        # Segnala che, per la missione in corso, il drone ha confermato il
        # rischio e il rover e' stato attivato: la missione non va chiusa
        # al rientro del drone ma solo al rientro del rover.
        self.risk_confirmed_sub = self.create_subscription(
            String, '/agro/risk_confirmed', self.on_risk_confirmed, 10)
        self.mission_cmd_pub = self.create_publisher(
            String, '/agro/mission_cmd', 10)
        self.event_pub = self.create_publisher(
            String, '/agro/events', 10)

        self._mission_active = False
        self._waiting_for_rover = False
        self._mission_counter = 0
        self._current_mission_id = None
        self._current_alert = None
        self._mission_start_epoch = None
        self._mission_start_iso = None

        # Coda FIFO delle allerte "high" arrivate a drone occupato:
        # [{"alert": ..., "enqueued_epoch": ...}, ...].
        # Si svuota di un elemento a ogni chiusura di missione. Nel ramo
        # "rischio non confermato" la chiusura e' il return_to_base del
        # drone, garantito anche in caso di guasto dal PHASE_TIMEOUT_S in
        # drone_controller_webots.py. Nel ramo "rischio confermato" serve
        # invece il return_to_base del rover, che non ha un timeout di
        # fase: se il rover si bloccasse, la coda non drenerebbe (limite
        # noto del prototipo).
        self._alert_queue = []

    def on_alert(self, msg: String):
        alert = json.loads(msg.data)
        risk_level = alert.get("risk_level")
        zone_id = alert.get("zone_id")

        if risk_level != "high":
            # Sotto soglia: non aziona una missione ne' va in coda, ma
            # resta comunque tracciata nel dataset 
            self._log_event(
                event_type="alert_received", alert=alert, mission_id=None,
                x=0.0, y=0.0, z=0.0,
                note=f"Allerta ricevuta, sotto soglia (risk_level={risk_level})")
            return

        if self._mission_active:
            if self._is_duplicate_zone(zone_id):
                self.get_logger().info(
                    f"Zona {zone_id} gia' in missione o in coda, "
                    "allerta duplicata non riaccodata.")
                self._log_event(
                    event_type="alert_duplicate", alert=alert, mission_id=None,
                    x=0.0, y=0.0, z=0.0,
                    note=(f"Zona {zone_id} gia' in missione o in coda, "
                          "allerta non riaccodata"))
                return

            self._enqueue_alert(alert)
            return

        self._log_event(
            event_type="alert_received", alert=alert, mission_id=None,
            x=0.0, y=0.0, z=0.0, note="Allerta ricevuta")
        self._start_mission(alert)

    def _is_duplicate_zone(self, zone_id):
        """Una zona e' considerata 'gia' gestita' se e' quella della
        missione in corso o se ha gia' un'allerta in coda."""
        if self._current_alert and self._current_alert.get("zone_id") == zone_id:
            return True
        return any(item["alert"]["zone_id"] == zone_id
                   for item in self._alert_queue)

    def _enqueue_alert(self, alert):
        enqueued_epoch = time.time()
        self._alert_queue.append({
            "alert": alert,
            "enqueued_epoch": enqueued_epoch,
        })
        self.get_logger().info(
            f"Allerta accodata: zona={alert.get('zone_id')}, "
            f"lunghezza coda={len(self._alert_queue)}")
        self._log_event(
            event_type="alert_queued", alert=alert, mission_id=None,
            x=0.0, y=0.0, z=0.0,
            note=(f"Allerta accodata (posizione {len(self._alert_queue)} "
                  f"in coda), drone occupato su "
                  f"{self._current_alert.get('zone_id')}"))

    def _dequeue_next_mission(self):
        if not self._alert_queue:
            return

        item = self._alert_queue.pop(0)
        alert = item["alert"]
        enqueued_epoch = item["enqueued_epoch"]
        dequeue_epoch = time.time()
        wait_seconds = round(dequeue_epoch - enqueued_epoch, 3)

        self.get_logger().info(
            f"Allerta sfilata dalla coda: zona={alert.get('zone_id')}, "
            f"lunghezza coda={len(self._alert_queue)}, "
            f"wait_seconds={wait_seconds}")
        self._log_event(
            event_type="queue_wait", alert=alert, mission_id=None,
            x=0.0, y=0.0, z=0.0,
            note=f"Uscita dalla coda dopo {wait_seconds}s di attesa, "
                 "missione avviata",
            timestamp_start=_iso_ms(enqueued_epoch),
            timestamp_end=_iso_ms(dequeue_epoch),
            duration_seconds=wait_seconds)

        self._start_mission(alert)

    def on_risk_confirmed(self, msg: String):
        payload = json.loads(msg.data)
        if (self._mission_active
                and payload.get("mission_id") == self._current_mission_id):
            self._waiting_for_rover = True
            self.get_logger().info(
                f"Rischio confermato per {self._current_mission_id}: la "
                "missione resta attiva fino al rientro del rover.")

    def on_event(self, msg: String):
        event = json.loads(msg.data)
        if (event.get("event_type") == "return_to_base"
                and event.get("mission_id") == self._current_mission_id):
            # Ramo "rischio confermato": il rientro del drone (o una sua
            # chiusura forzata) non conclude la missione. Si attende il
            # return_to_base del rover (source == "rover"). Se il rover
            # restasse bloccato la missione non si chiude: limite noto del
            # prototipo, preferito a un avvio prematuro della missione
            # successiva con lo stato della piattaforma ancora incerto.
            if self._waiting_for_rover and event.get("source") != "rover":
                self.get_logger().info(
                    f"Rientro del drone per {self._current_mission_id} "
                    "registrato; in attesa del rientro del rover.")
                return

            end_iso = event["timestamp_end"]
            end_epoch = datetime.fromisoformat(end_iso).timestamp()
            duration = round(end_epoch - self._mission_start_epoch, 3)

            self._log_event(
                event_type="mission",
                alert=self._current_alert, mission_id=self._current_mission_id,
                x=event.get("x", 0.0), y=event.get("y", 0.0),
                z=event.get("z", 0.0), note="Missione conclusa",
                timestamp_start=self._mission_start_iso,
                timestamp_end=end_iso, duration_seconds=duration)

            self._mission_active = False
            self._waiting_for_rover = False
            self.get_logger().info(
                f"Missione {self._current_mission_id} completata "
                f"({duration}s), pronto per la prossima allerta.")

            self._dequeue_next_mission()

    def _start_mission(self, alert):
        self._mission_active = True
        self._waiting_for_rover = False
        self._mission_counter += 1
        mission_id = f"M{self._mission_counter:03d}"
        zone_id = alert["zone_id"]
        x, y = ZONE_POSITIONS.get(zone_id, (0.0, 0.0))

        self._current_mission_id = mission_id
        self._current_alert = alert
        self._mission_start_epoch = time.time()
        self._mission_start_iso = _iso_ms(self._mission_start_epoch)

        cmd = {
            "mission_id": mission_id,
            "alert_id": alert["alert_id"],
            "zone_id": zone_id,
            "drone_id": "drone_1",
            "target_x": x,
            "target_y": y,
            "target_z": 2.0,
            "action": alert.get("recommended_action", "localized_treatment"),
        }
        cmd_msg = String()
        cmd_msg.data = json.dumps(cmd)
        self.mission_cmd_pub.publish(cmd_msg)
        self.get_logger().info(f"Missione avviata: {cmd_msg.data}")

    def _log_event(self, event_type, alert, mission_id, x, y, z, note,
                    timestamp_start=None, timestamp_end=None,
                    duration_seconds=None):
        if timestamp_start is None and timestamp_end is None:
            # evento istantaneo (es. alert_received): inizio == fine
            timestamp_start = timestamp_end = _iso_ms()
            duration_seconds = 0.0

        event = {
            "timestamp_start": timestamp_start,
            "timestamp_end": timestamp_end,
            "duration_seconds": duration_seconds,
            "event_type": event_type,
            "alert_id": alert.get("alert_id"),
            "mission_id": mission_id,
            "zone_id": alert.get("zone_id"),
            "drone_id": "drone_1",
            "risk_level": alert.get("risk_level"),
            "pest": alert.get("pest"),
            "action": alert.get("recommended_action"),
            "source": "manager",
            "x": x, "y": y, "z": z,
            "note": note,
        }
        e_msg = String()
        e_msg.data = json.dumps(event)
        self.event_pub.publish(e_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
