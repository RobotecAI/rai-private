# Contributing

We are very happy you want to contribute to RAI, and we welcome all input.
This document outlines guidelines for contributors to follow.

Our philosophy is strongly aligned with the philosophy of the ROS development process.
These guidelines have been therefore strongly influenced by the [ROS2 Contributing document](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing.html).

## Tenets

- Engage Robotec.ai as early as possible

  - Start discussions with Robotec.ai and the community early. Long time RAI contributors may have a clearer vision of the big picture. If you implement a feature and send a pull request without discussing with the community first, you are taking the risk of it being rejected, or you may be asked to largely rethink your design.
  - Opening issues or using Discourse to socialize an idea before starting the implementation is generally preferable.

- Adopt community best-practices whenever possible instead of ad-hoc processes

  Think about the end-users experience when developing and contributing. Features accessible to a larger amount of potential users, utilising widely available solutions are more likely to be accepted.

- Think about the community as a whole

  Think about the bigger picture. There are developers building different robots with different constraints. The landscape of available AI models is rapidly changing, coming with different capabilities and constraints. RAI wants to accommodate requirements of the whole community.

There are a number of ways you can contribute to the RAI project.

## Discussions and support

Some of the easiest ways to contribute to RAI involve engaging in community discussions and support. This can be done by creating Issues and RFCs on github.

## Contributing code

### Setting up the development environment

Set up your development environment following [the instructions](docs/developer_guide.md#developer-environment-setup).

### Starting the discussion

Always try to engage in discussion first. Browse Issues and RFCs or start a discussion on [RAI Discord](https://discord.gg/GZGfejUSjt) to see if a feature you want to propose (or a similar one) has already been mentioned.
If that is the case feel free to offer that you'll work on it, and propose what changes/additions you will make.
One of the project maintainers will assign the issue to you, and you can start working on the code.

### Submitting code changes

To submit a change begin by forking this repository and making the changes on the fork.
Once the changes are ready to be proposed create a pull request back to the repository.
In order to maintain a linear and clear commit history please:

- make sure that all commits have meaningful messages
- if batches of "cleanup" or similar commits are present - squash them together
- rebase onto the main branch of repository before making the PR

Always make sure that both all tests are passing before making the PR.
Once this is done open your PR describing what changes have been made and how to test if it's working.
Request review from the maintainer who assigned you the issue.

If the review requires modifications please make them on your forked repository and go through the above process.
Once the PR is accepted it will be merged into the repository.
Congratulations, and thank you for your contributions to the development of RAI.
