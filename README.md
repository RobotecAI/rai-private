# RAI

> [!IMPORTANT]  
> **RAI is in beta phase now, expect friction. Early contributors are the most welcome!** \
> **RAI is developing fast towards a glorious release in time for ROSCon 2024.**

![rai-image](./docs/imgs/RAI_simple_diagram_medium.png)

<div align="center">

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
![GitHub Release](https://img.shields.io/github/v/release/RobotecAI/rai)
![Contributors](https://img.shields.io/github/contributors/robotecai/rai)

![Static Badge](https://img.shields.io/badge/Ubuntu-24.04-orange)
![Static Badge](https://img.shields.io/badge/Ubuntu-22.04-orange)
![Static Badge](https://img.shields.io/badge/Python-3.12-blue)
![Static Badge](https://img.shields.io/badge/Python-3.10-blue)
![Static Badge](https://img.shields.io/badge/ROS2-jazzy-blue)
![Static Badge](https://img.shields.io/badge/ROS2-humble-blue)

[![](https://dcbadge.limes.pink/api/server/https://discord.gg/GZGfejUSjt)](https://discord.gg/GZGfejUSjt)

</div>

## Overview

The RAI framework aims to:

- Supply a general multi-agent system, bringing Gen AI features to your robots.
- Add human interactivity, flexibility in problem-solving, and out-of-box AI features to existing robot stacks.
- Provide first-class support for multi-modalities, enabling interaction with various data types.
- Incorporate an advanced database for persistent agent memory.
- Include ROS 2-oriented tooling for agents.
- Support a comprehensive task/mission orchestrator.

# Table of Contents

- [Features](#features)
- [Quick Start](#quick-start)
- [Developer guide](./docs/developer_guide.md)
- [Usage examples (demos)](#planned-demos)
- [Further documentation](#further-documentation)
- [ROSCon 2024 Talk](#roscon-2024)

## Features

- [x] Voice interaction (both ways).
- [x] Customizable robot identity, including constitution (ethical code) and documentation (understanding own capabilities).
- [x] Accessing camera ("What do you see?") sensor, utilizing VLMs.
- [x] Reasoning about its own state through ROS logs.
- [x] ROS 2 action calling and other interfaces. The Agent can dynamically list interfaces, check their message type, and publish.
- [x] Integration with LangChain to abstract vendors and access convenient AI tools.
- [x] Tasks in natural language to nav2 goals.
- [ ] NoMaD integration.
- [ ] OpenVLA integration.
- [ ] Improved Human-Robot Interaction with voice and text.
- [ ] SDK for RAI developers.
- [ ] Support for at least 3 different AI vendors.
- [ ] Additional tooling such as GroundingDino.
- [ ] UI for configuration to select features and tools relevant for your deployment.

# Quick Start

Currently, RAI supports Ubuntu 24.04 with ROS 2 Jazzy and Python 3.12.
The release will also support Ubuntu 22.04 with ROS 2 Humble (it should work or be close to working now).

### 1. Setting up the workspace:

#### 1.1 Install poetry

Install poetry (1.8+) with the following line, or

```bash
curl -sSL https://install.python-poetry.org | python3 -
```

by following the official [docs](https://python-poetry.org/docs/#installation)

#### 1.2 Clone the repository:

```bash
git clone https://github.com/RobotecAI/rai.git
cd rai
```

#### 1.3 Create poetry virtual environment and install dependencies:

```bash
poetry install
rosdep install --from-paths src --ignore-src -r -y
```

### 2. Build the project:

#### 2.1 Build RAI workspace

```bash
colcon build --symlink-install
```

#### 2.2 Activate a virtual environment:

```bash
source ./setup_shell.sh
```

### 3. Setting up vendors

RAI is fully vendor-agnostic, however the beta development work currently utilizes OpenAI models. Setting the `OPENAI_API_KEY` environment variable will yield the best results.

#### OpenAI

If you do not have a key, see how to generate one [here](https://platform.openai.com/docs/quickstart).

```
export OPENAI_API_KEY=""
```

#### Congratulations, your installation is now complete! Head to [Running example](./docs/developer_guide.md)

# Running RAI

![rosbot-xl-example](./docs/imgs/rosbot-xl-example.gif)

RAI is a sophisticated framework targeted at solving near general cases. As of now, we provide the following examples:

1. Engage with your ROS 2 network through an intuitive Streamlit chat interface.
2. Explore the O3DE Husarion ROSbot XL demo and assign tasks via natural language.

If you are more ambitious:

- Create your own robot description package and unleash it with the rai_whoami node.
- Run Streamlit powered by your custom robot’s description package and effortlessly access your robot's documentation as well as identity and constitution.
- Implement additional tools via LangChain's @tool and use them in your chat.

## 1. Chat Interface

Chat seamlessly with your setup, retrieve images from cameras, adjust parameters on the fly, and get comprehensive information about your topics.

```bash
streamlit run src/rai_hmi/rai_hmi/streamlit_hmi_node.py
```

Remember to run this command in a sourced shell.

## 2. O3DE Rosbot XL Demo

This demo provides a practical way to interact with and control a virtual Husarion ROSbot XL within a simulated environment.
Using natural language commands, you can assign tasks to the robot, allowing it to perform a variety of actions.

Given that this is a beta release, consider this demo as an opportunity to explore the framework's capabilities, provide feedback, and contribute.
Try different commands, see how the robot responds, and use this experience to understand the potential and limitations of the system.

Follow this guide: [husarion-rosbot-xl-demo](./docs/demos.md)

## Planned demos

| Application                                | Robot                          | Description                                                                                                                                      | Link                                                          |
| ------------------------------------------ | ------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------- |
| Mission and obstacle reasoning in orchards | Autonomous tractor             | In a beautiful scene of a virtual orchard, RAI goes beyond obstacle detection to analyze best course of action for a given unexpected situation. | [🌾 demo](https://github.com/RobotecAI/rai-agriculture-demo)  |
| Manipulation tasks with natural language   | Robot Arm (Franka Panda)       | Complete flexible manipulation tasks thanks to RAI and OpenVLA                                                                                   | [🦾 demo](https://github.com/RobotecAI/rai-manipulation-demo) |
| Quadruped inspection demo                  | A robot dog (ANYbotics ANYmal) | Perform inspection in a warehouse environment, find and report anomalies                                                                         | link TBD                                                      |

# Community

## RAI Q&A

Please take a look at [Q&A](https://github.com/RobotecAI/rai/discussions/categories/q-a).

## Developer Resources

See our [Developer Guide](docs/developer_guide.md).

## Contributing

You are welcome to contribute to RAI! Please see our [Contribution Guide](CONTRIBUTING.md).

## RAI release and talk

RAI will be released on **October 15th**, right before [ROSCon 2024](https://roscon.ros.org/2024/).
If you are going to the conference, come join us at RAI talk on October 23rd.

<p align="center">
<img width="400" src="./docs/imgs/talk.png" />
</p>
