# Copyright (C) 2024 Robotec.AI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# The contents of this file include code from (https://github.com/robodhruv/visualnav-transformer) as well as fork (https://github.com/RobotecAI/visualnav-transformer-ros2) with the following license:
# MIT License

# Copyright (c) 2023 Dhruv Shah, Ajay Sridhar, Nitish Dashora, Kyle Stachowicz, Kevin Black, Noriaki Hirose, Sergey Levine

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

import os
from typing import Tuple

import gdown
import numpy as np
import rclpy
import torch
import yaml
from ament_index_python.packages import get_package_share_directory
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty
from visualnav_transformer.deployment.src.utils import (
    load_model,
    msg_to_pil,
    to_numpy,
    transform_images,
)
from visualnav_transformer.train.vint_train.training.train_utils import get_action


class NomadNode(Node):
    def __init__(self):
        super().__init__("rai_nomad_node")

        self._initialize_parameters()
        self._initialize_nomad()

        self.create_service(Empty, "/rai_nomad/start", self.start_callback)
        self.create_service(Empty, "/rai_nomad/stop", self.stop_callback)

        self.image_subscription = None

        cmd_vel_topic = (
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        )
        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)

    def _initialize_parameters(self):
        self.declare_parameter(
            "model_path",
            os.path.join(get_package_share_directory("rai_nomad"), "nomad.pth"),
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description=(
                    "Path to the .pth file containing the nomad model weights (will be downloaded if not present)"
                ),
            ),
        )
        self.declare_parameter(
            "model_config_path",
            os.path.join(get_package_share_directory("rai_nomad"), "nomad_params.yaml"),
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description=(
                    "Path to the .yaml file containing the nomad model configuration"
                ),
            ),
        )
        self.declare_parameter(
            "image_topic",
            "/camera/color/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description=("The topic to subscribe to for image data"),
            ),
        )
        self.declare_parameter(
            "cmd_vel_topic",
            "/cmd_vel",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description=("The topic to publish velocity commands to"),
            ),
        )
        self.declare_parameter(
            "sampled_actions_topic",
            "/sampled_actions",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description=("The topic to publish sampled actions to"),
            ),
        )
        self.declare_parameter(
            "linear_vel",
            0.01,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description=("Linear velocity scaling of the robot"),
            ),
        )
        self.declare_parameter(
            "angular_vel",
            3.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description=("Angular velocity scaling of the robot"),
            ),
        )
        self.declare_parameter(
            "max_v",
            0.2,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description=("Maximum linear velocity of the robot"),
            ),
        )
        self.declare_parameter(
            "max_w",
            1.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description=("Maximum angular velocity of the robot"),
            ),
        )
        self.declare_parameter(
            "rate",
            6,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description=("Rate at which the model is run"),
            ),
        )

    def _download_model(self, ckpth_path):
        gdown.download(id="1YJhkkMJAYOiKNyCaelbS_alpUpAJsOUb", output=ckpth_path)

    def _initialize_nomad(self):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Using device: {self.device}")

        self.sampled_actions_pub = self.create_publisher(
            Float32MultiArray,
            self.get_parameter("sampled_actions_topic")
            .get_parameter_value()
            .string_value,
            1,
        )

        self.context_queue = []
        self.timer = None

        model_config_path = (
            self.get_parameter("model_config_path").get_parameter_value().string_value
        )
        with open(model_config_path, "r") as f:
            self.model_params = yaml.safe_load(f)

        self.context_size = self.model_params["context_size"]

        # load model weights
        ckpth_path = self.get_parameter("model_path").get_parameter_value().string_value
        if os.path.exists(ckpth_path):
            self.get_logger().info(f"Loading model from {ckpth_path}")
        else:
            self._download_model(ckpth_path)
        self.model = load_model(
            ckpth_path,
            self.model_params,
            self.device,
        )
        self.model = self.model.to(self.device)
        self.model.eval()

        self.num_diffusion_iters = self.model_params["num_diffusion_iters"]
        self.noise_scheduler = DDPMScheduler(
            num_train_timesteps=self.model_params["num_diffusion_iters"],
            beta_schedule="squaredcos_cap_v2",
            clip_sample=True,
            prediction_type="epsilon",
        )

    def start_callback(self, request, response):
        if self.image_subscription is not None:
            self.get_logger().warn(
                "Start service called, but the model is already running!"
            )
            return response

        self.get_logger().info("Start service called")
        rate = self.get_parameter("rate").get_parameter_value().integer_value
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, depth=10
        )
        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.image_subscription = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile
        )
        return response

    def stop_callback(self, request, response):
        if self.image_subscription is None:
            self.get_logger().warn("Stop service called, but the model is not running!")
            return response

        self.get_logger().info("Stop service called")
        self.destroy_subscription(self.image_subscription)
        self.image_subscription = None
        self.timer.cancel()
        self.timer = None
        self.context_queue = []
        return response

    def image_callback(self, msg):
        obs_img = msg_to_pil(msg)
        if self.context_size is not None:
            if len(self.context_queue) < self.context_size + 1:
                self.context_queue.append(obs_img)
            else:
                self.context_queue.pop(0)
                self.context_queue.append(obs_img)

    def timer_callback(self):
        waypoint_msg = Float32MultiArray()
        if len(self.context_queue) > self.model_params["context_size"]:
            obs_images = transform_images(
                self.context_queue, self.model_params["image_size"], center_crop=False
            )
            obs_images = obs_images.to(self.device)
            fake_goal = torch.randn((1, 3, *self.model_params["image_size"])).to(
                self.device
            )
            mask = torch.ones(1).long().to(self.device)  # ignore the goal

            # infer action
            with torch.no_grad():
                # encoder vision features
                obs_cond = self.model(
                    "vision_encoder",
                    obs_img=obs_images,
                    goal_img=fake_goal,
                    input_goal_mask=mask,
                )

                # (B, obs_horizon * obs_dim)
                NUM_SAMPLES = 8
                if len(obs_cond.shape) == 2:
                    obs_cond = obs_cond.repeat(NUM_SAMPLES, 1)
                else:
                    obs_cond = obs_cond.repeat(NUM_SAMPLES, 1, 1)

                # initialize action from Gaussian noise
                noisy_action = torch.randn(
                    (NUM_SAMPLES, self.model_params["len_traj_pred"], 2),
                    device=self.device,
                )
                naction = noisy_action

                # init scheduler
                self.noise_scheduler.set_timesteps(self.num_diffusion_iters)

                for k in self.noise_scheduler.timesteps[:]:
                    # predict noise
                    noise_pred = self.model(
                        "noise_pred_net",
                        sample=naction,
                        timestep=k,
                        global_cond=obs_cond,
                    )

                    # inverse diffusion step (remove noise)
                    naction = self.noise_scheduler.step(
                        model_output=noise_pred, timestep=k, sample=naction
                    ).prev_sample

            naction = to_numpy(get_action(naction))
            sampled_actions_msg = Float32MultiArray()
            sampled_actions_msg.data = np.concatenate(
                (np.array([0]), naction.flatten())
            ).tolist()
            self.sampled_actions_pub.publish(sampled_actions_msg)

            naction = naction[0]

            chosen_waypoint = naction[2]

            if self.model_params["normalize"]:
                max_v = self.get_parameter("max_v").get_parameter_value().double_value
                rate = self.get_parameter("rate").get_parameter_value().integer_value
                chosen_waypoint *= max_v / rate

            waypoint_msg.data = chosen_waypoint.tolist()
            v, w = self.pd_controller(waypoint_msg.data)
            twist = Twist()
            twist.linear.x = v
            twist.angular.z = w
            self.publisher.publish(twist)

    def pd_controller(self, waypoint: np.ndarray) -> Tuple[float]:
        """PD controller for the robot"""
        EPS = 1e-8
        linear_vel = self.get_parameter("linear_vel").get_parameter_value().double_value
        angular_vel = (
            self.get_parameter("angular_vel").get_parameter_value().double_value
        )
        assert (
            len(waypoint) == 2 or len(waypoint) == 4
        ), "waypoint must be a 2D or 4D vector"
        if len(waypoint) == 2:
            dx, dy = waypoint
        else:
            dx, dy, hx, hy = waypoint
        # this controller only uses the predicted heading if dx and dy near zero
        if np.abs(dx) < EPS:
            v = 0
            w = np.sign(dy) * np.pi
        else:
            v = linear_vel * dx / np.abs(dy)
            w = angular_vel * np.arctan(dy / dx)
        max_v = self.get_parameter("max_v").get_parameter_value().double_value
        max_w = self.get_parameter("max_w").get_parameter_value().double_value
        v = np.clip(v, -max_v, max_v)
        w = np.clip(w, -max_w, max_w)
        return v, w

        print("Published waypoint")


def main(args=None):
    rclpy.init(args=args)
    node = NomadNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
