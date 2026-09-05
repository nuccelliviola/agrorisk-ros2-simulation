"""
Nodo ROS2 che registra gli eventi pubblicati su /agro/events in un file CSV.

Il percorso è configurabile tramite il parametro `log_path`. Se non viene
specificato, viene creato un nuovo file in ~/agrorisk_dataset con timestamp
nel nome. L'intestazione viene scritta quando il file è nuovo o vuoto.
"""
import json
import csv
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


FIELDNAMES = [
    "timestamp_start", "timestamp_end", "duration_seconds", "event_type",
    "alert_id", "mission_id", "zone_id", "drone_id", "source",
    "risk_level", "pest",
    "indice_stress",
    "action", "x", "y", "z", "note",
]

DATASET_DIR = Path.home() / 'agrorisk_dataset'


class MissionLogger(Node):
    def __init__(self):
        super().__init__('mission_logger')
        self.declare_parameter('log_path', '')
        log_path = self.get_parameter('log_path').value

        if log_path:
            self._log_path = Path(log_path)
        else:
            timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            self._log_path = DATASET_DIR / f'log_{timestamp}.csv'

        self._log_path.parent.mkdir(parents=True, exist_ok=True)

        self._write_header_if_needed()

        self.sub = self.create_subscription(
            String, '/agro/events', self.on_event, 10)
        self.get_logger().info(
            f"Dataset di log salvato in: {self._log_path}")

    def _write_header_if_needed(self):
        if not self._log_path.exists() or self._log_path.stat().st_size == 0:
            with open(self._log_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
                writer.writeheader()

    def on_event(self, msg: String):
        event = json.loads(msg.data)
        row = {k: event.get(k, "") for k in FIELDNAMES}
        with open(self._log_path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
            writer.writerow(row)
        self.get_logger().info(f"Evento salvato: {row['event_type']}")


def main(args=None):
    rclpy.init(args=args)
    node = MissionLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
