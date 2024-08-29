import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

STATE_TO_COLOR = {
    "waiting": (255, 255, 255),  # white
    "processing": (255, 134, 0),  # yellow
    "recording": (0, 255, 0),  # green
    "playing": (0, 0, 255),  # blue
}
DEFAULT_COLOR = (255, 0, 0)  # red, unknown state
PULSE_FREQUENCY = 0.5  # Hz


class LEDStripController(Node):

    def __init__(self):
        super().__init__("led_strip_controller")
        self.asr_state = "waiting"
        self.hmi_state = "waiting"
        self.tts_state = "waiting"

        self.create_subscription(String, "/asr_status", self.asr_status_callback, 10)
        self.create_subscription(String, "/hmi_status", self.hmi_status_callback, 10)
        self.create_subscription(String, "/tts_status", self.tts_status_callback, 10)

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.publisher_ = self.create_publisher(Image, "/led_strip", 10)

        self.led_state = "waiting"

    def asr_status_callback(self, msg: String) -> None:
        if isinstance(msg.data, str):
            self.asr_state = msg.data
            self.calculate_state()

    def hmi_status_callback(self, msg: String) -> None:
        if isinstance(msg.data, str):
            self.hmi_state = msg.data
            self.calculate_state()

    def tts_status_callback(self, msg: String) -> None:
        if isinstance(msg.data, str):
            self.tts_state = msg.data
            self.calculate_state()

    def calculate_state(self) -> str:
        # priority order: recording > playing > processing > waiting
        if self.asr_state == "recording" and self.tts_state == "playing":
            self.get_logger().warn(
                "ASR is recording and TTS is playing at the same time!"
            )
            return ""

        if self.asr_state == "recording" and self.hmi_state == "processing":
            self.get_logger().warn(
                "ASR is recording and HMI is processing at the same time!"
            )
            return ""

        if self.led_state == "waiting":
            if self.tts_state == "playing":
                self.led_state = "playing"
            elif self.asr_state == "recording":
                self.led_state = "recording"
        elif self.led_state == "recording":
            if self.asr_state == "transcribing":
                self.led_state = "processing"
        elif self.led_state == "processing":
            if self.tts_state == "playing":
                self.led_state = "playing"
            elif self.asr_state == "dropping":
                self.led_state = "waiting"
        elif self.led_state == "playing":
            if self.tts_state == "waiting":
                self.led_state = "waiting"

    def timer_callback(self):
        color = STATE_TO_COLOR.get(self.led_state, DEFAULT_COLOR)

        if self.led_state == "playing":
            t = self.get_clock().now().nanoseconds / 1e9
            value: float = 0.1 + 0.45 * (np.sin(2 * np.pi * PULSE_FREQUENCY * t) + 1.0)
            color = np.array(color)
            color = (value * color).astype(np.uint8)

        led_colors = np.full((1, 18, 3), color, dtype=np.uint8)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = 18
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = 18 * 3
        msg.data = led_colors.flatten().tolist()

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LEDStripController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
