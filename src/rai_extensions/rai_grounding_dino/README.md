# RAI GroundingDINO

This package provides a ROS2 Node which is an interface to the [Idea-Research GroundingDINO Model](https://github.com/IDEA-Research/GroundingDINO).
It allows for open-set detection.

## Installation

In your workspace you need to have an `src` folder containing this package `rai_grounding_dino` and the `rai_interfaces` package.

### Preparing the GroundingDINO

Add required ROS dependencies:

```
rosdep install --from-paths src --ignore-src -r
```

### Build and run

In the base directory of the `RAI` package install dependencies:

```
poetry install --with gdino
```

Source the ros installation

```
source /opt/ros/${ROS_DISTRO}/setup.bash
```

Run the build process:

```
colcon build
```

Source the local installation:

```
source ./install/setup.bash
```

Activate the poetry environment:

```
poetry shell
export PYTHONPATH="$(dirname $(dirname $(poetry run which python)))/lib/python$(poetry run python --version | awk '{print $2}' | cut -d. -f1,2)/site-packages:$PYTHONPATH"
```

Run the ROS node using `ros2 launch`:

```
ros2 launch rai_grounding_dino gdino_launch.xml [weights_path:=PATH/TO/WEIGHTS]
```

> [!NOTE]
> By default the weights will be downloaded to `$(ros2 pkg prefix rai_grounding_dino)/share/weights/`.
> You can change this path if you downloaded the weights manually or moved them.

### Example

An example client is provided with the package as `rai_grounding_dino/talker.py`

You can see it working by running:

```
ros2 launch rai_grounding_dino example_communication_launch.xml image_path:=src/rai_extensions/rai_grounding_dino/images/sample.jpg [weights_path:=PATH/TO/WEIGHTS]
```

If everything was set up properly you should see a couple of detections with classes `dinosaur`, `dragon`, and `lizard`.
