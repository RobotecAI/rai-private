# Robot arm manipulation

## Table of Contents

- [Overview](#overview)
- [Currently planned and developed pipeline](#currently-planned-and-developed-pipeline)
  - [Automated Dataset Generation](#automated-dataset-generation)
  - [OpenVLA fine-tuning and inference](#openvla-fine-tuning-and-inference)
- [Summary of Fine-Tuning Experiments](#summary-of-fine-tuning-experiments)
  - [Experiments](#experiments)
  - [Challenges and Limitations](#challenges-and-limitations)
  - [Conclusion](#conclusion)

## Overview

Vision-Language-Action (VLA) models represent a novel approach to controlling robots,
offering significant potential to automate and generalize robotic operations.
We are working on integrating this approach with RAI to further extend RAI's versatility.

The upcoming manipulation demo will showcase a practical use case where a single prompt can be used to instruct a robotic arm to perform a series of specific tasks.
It is also meant to showcase how a more generalist agent cooperates with a specialized model.

## Currently planned and developed pipeline

![pipeline](imgs/openvla_diagram.gif)

### **Automated Dataset Generation**

To automatically generate a dataset with various scenes and robot's actions to fine-tune VLA (Vision-Language-Action) models for generalist robot manipulation policies,
we leverage our [**LLM-powered Scene Generator**](https://github.com/RobotecAI/o3de-genai-gems) enabling easy generation of various scenarios based on user prompts.

**LLM-powered Scene Generator** is an [Open 3D Engine (O3DE)](https://o3de.org/industries/robotics-and-simulations/) Gem that generates the Python code that creates the prompted scene and makes objects make desired actions.
We use the most efficient [Claude models](https://www.anthropic.com/claude) (currently Claude 3.5 Sonnet).

![manipulation_examples](imgs/manipulation_demo.gif)

### OpenVLA fine-tuning and inference

We integrated [openVLA](https://openvla.github.io/) model with O3DE and ROS 2.
Our work on Robotic Arm Manipulation Demo is public and available [here](https://github.com/RobotecAI/rai-manipulation-demo).
Instructions on how to run the demo will be released soon.

## Summary of Fine-Tuning Experiments

Our team conducted a series of fine-tuning experiments using our custom dataset from simulation, focusing on a one simple task. This novel approach presented both exciting opportunities and significant challenges.

### Experiments

We explored various aspects of the dataset and training process, including:

1. Different camera positions
2. Varying dataset sizes
3. Multiple episode frequencies
4. Diverse starting positions for the robotic arm within episodes
   - This was aimed at enhancing the model's ability to continue tasks even when the arm overshoots the target object

### Challenges and Limitations

During the experiments we encountered several obstacles and challanges:

1. **Computational Requirements**:

   - OpenVLA fine-tuning demands substantial computational resources, as noted in the [official documentation](https://github.com/openvla/openvla?tab=readme-ov-file#fine-tuning-openvla-via-lora).
   - While fine-tuning is possible with limited resources, it significantly impacts efficiency:
     - Using 2 x RTX 4090 GPUs (48 GB VRAM total), model convergence took approximately 5 days with a batch size of 2.
     - In contrast, a single H100 GPU (80 GB VRAM) achieved convergence in about 2 hours with a batch size of 8.

2. **Metric Limitations**:

   - The current metrics implemented in the openVLA repository proved to be inadequate indicators of the model's true success rate.
   - For instance, achieving near 100% training accuracy did not correlate with real-world performance, even on training samples.

3. **Novelty of Approach**:
   - OpenVLA represents a cutting-edge technique in the field, requiring extensive experimentation to develop the necessary intuitions for successful fine-tuning.

### Conclusion

While our experiments with openVLA did not yield the desired results or significant progress at this stage, they have provided valuable insights into the challenges and potential of this approach. The complexity of the task and the novelty of the method underscore the need for continued research and development in this area.
