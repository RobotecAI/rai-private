# RAI NoMaD

This package provides a ROS2 Node which loads and runs the [NoMaD](https://general-navigation-models.github.io/nomad/index.html) model, that can be dynamically activated and deactivated using ROS2 messages.

## Running instructions

### 1. Setup the ROS2 workspace

In the base directory of the `RAI` package install dependancies:

```
poetry install --with nomad
```

Source the ros installation

```
source /opt/ros/${ROS_DISTRO}/setup.bash
```

Run the build process:

```
colcon build
```

Setup your shell:

```
source ./setup_shell.sh
```

### 2. Run the node

Run the ROS2 node using `ros2 run`:

```
ros2 run rai_nomad nomad --ros-args -p image_topic:=<your_image_topic>
```

The model will be loaded and ready, but it will not run until you call the `/rai_nomad/start` service. Then it will start outputting velocity commands and your robot should start moving. You can then stop the model by calling the `/rai_nomad/stop` service.

### Parameters

- `model_path`: Path to where the model should be downloaded. By default it will be downloaded into the package's `share` directory.
- `image_topic`: The topic where the camera images are being published.
- `cmd_vel_topic`: The topic where the velocity commands are being published. Default: `/cmd_vel`.
- `linear_vel`: Linear velocity scaling of the model output.
- `angular_vel`: Angular velocity scaling of the model output.
- `max_v` and `max_w`: Maximum linear and angular velocities that the model can output.
- `rate`: The rate at which the model will run.
