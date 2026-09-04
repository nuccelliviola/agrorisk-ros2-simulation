"""
Nodo che pubblica una sequenza finita e deterministica di allerte
AgroRisk per la demo, poi si ferma da solo. Ogni allerta della sequenza viene pubblicata una sola volta,
nell'ordine e con i ritardi definiti in ALERT_SEQUENCE; dopo l'ultima
non c'è nessuna ripetizione.


"""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


# Unica piaga gestita in questa demo 
PEST = "tuta_absoluta"

# Sequenza fissa della demo: ogni voce e' 
# -ritardo dall'evento precedente in secondi
# -zone_id, 
# -risk_level
# Il ritardo della prima voce e' dall'avvio del nodo: 15s per dare tempo a Webots di
# caricare il mondo e a tutti i nodi di completare la discovery prima
# che parta la prima allerta, cosi' il drone_controller e' gia' in
# ascolto e non si perde la missione iniziale.
ALERT_SEQUENCE = [
    (15.0, "ZONA_B", "high"),
    (15.0, "ZONA_C", "high"),
    (15.0, "ZONA_A", "low"),
    (15.0, "ZONA_D", "medium"),
]


class AgroAlertPublisher(Node):
    def __init__(self):
        super().__init__('agro_alert_publisher')

        self.publisher_ = self.create_publisher(String, '/agro/alert', 10)
        self.done_publisher_ = self.create_publisher(
            Bool, '/agro/alerts_done', 10)

        self._alert_counter = 0
        self._sequence_index = 0
        # Un solo timer alla volta: ricreato ad ogni evento con il
        # ritardo giusto, non ricreato dopo l'ultimo - niente ciclo.
        self._timer = None

        self._schedule_next()

    def _schedule_next(self):
        delay, _, _ = ALERT_SEQUENCE[self._sequence_index]
        self._timer = self.create_timer(delay, self._on_timer)

    def _on_timer(self):
        self._timer.cancel()
        self.destroy_timer(self._timer)
        self._timer = None

        _, zone_id, risk_level = ALERT_SEQUENCE[self._sequence_index]
        self._publish_alert(zone_id, risk_level)

        self._sequence_index += 1
        if self._sequence_index < len(ALERT_SEQUENCE):
            self._schedule_next()
        else:
            self._publish_done()

    def _publish_alert(self, zone_id, risk_level):
        self._alert_counter += 1
        alert = {
            "alert_id": f"A{self._alert_counter:03d}",
            "zone_id": zone_id,
            "risk_level": risk_level,
            "pest": PEST,
            "recommended_action": "localized_treatment",
        }
        msg = String()
        msg.data = json.dumps(alert)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Allerta pubblicata: {msg.data}")

    def _publish_done(self):
        msg = Bool()
        msg.data = True
        self.done_publisher_.publish(msg)
        self.get_logger().info("Sequenza allerte completata")


def main(args=None):
    rclpy.init(args=args)
    node = AgroAlertPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
