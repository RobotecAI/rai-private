import json
from collections import defaultdict
from functools import partial

import rclpy
import rclpy.qos
import std_srvs.srv
import tf2_msgs.msg
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict


def callback(data, topic_name, msg):
    data[topic_name].append(msg)


class RaiState(Node):
    def __init__(self):
        super().__init__("rai_state")
        self.create_service(std_srvs.srv.Trigger, "/rai/state", self.get_state)
        self.create_service(
            std_srvs.srv.Trigger, "/rai/state/configure", self.configure
        )
        self.subscribers = list()
        self.data = defaultdict(list)

    def configure(self, request, response):
        watch_topics = ["/tf"]
        for t in watch_topics:
            subscriber = self.create_subscription(
                msg_type=tf2_msgs.msg.TFMessage,
                topic=t,
                callback=partial(callback, self.data, t),
                qos_profile=rclpy.qos.qos_profile_sensor_data,
            )
            self.subscribers.append(subscriber)

        self.get_logger().info(f"Configured: {watch_topics=}")
        return response

    def get_state(self, request, response):
        response.success = True
        data = dict()
        data["watched_topics"] = list()
        for topic_name, msg_list in self.data.items():
            msg_dict = message_to_ordereddict(msg_list[-1])
            data["watched_topics"].append({topic_name: (len(msg_list), msg_dict)})
            response.message = json.dumps(data)
        self.get_logger().info(f"State: {response.message}")
        return response


if __name__ == "__main__":
    rclpy.init()
    node = RaiState()
    rclpy.spin(node)
    rclpy.shutdown()
