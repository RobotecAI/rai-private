# ROS Packages

RAI comes with multiple configurable ROS2 packages which can be installed alongside the main distribution.

## RAI_asr

The RAI ASR (Automatic Speech Recognition) node utilizes a combination of voice activity detection (VAD) and a speech recognition model to transcribe spoken language into text using [openai-whisper](https://github.com/openai/whisper).

Detailed documentation and installation instructions are available in package [README](../src/rai_asr/README.md)

## RAI_grounding_dino

Package enabling use of [GroundingDINO](https://github.com/IDEA-Research/GroundingDINO) -- an open-set detection model with ROS2.

Detailed documentation and installation instructions are available in package [README](../src/rai_extensions/rai_grounding_dino/README.md)

## RAI_interfaces

Package containing definition of custom messages and services used in RAI. Should be used as a dependancy if any interfaces are required in another package.
To use add `<exec_depend>rai_interfaces</exec_depend>` to target package's `package.xml`.

## RAI_whoami

Package with robot-self identification capabilities. It includes RAI constitution.

## RAI_bringup

Package with launch files.

### Human - Robot interface via voice

```
ros2 launch rai_bringup hri.launch.py  tts_vendor:=(opentts|elevenlabs) robot_package_description:=(robot_whoami_package) recording_device:=0 keep_speaker_busy:=(true|false)
```

recording_device: The device you want to record with. Check available with:

```bash
python -c 'import sounddevice as sd; print(sd.query_devices())'
```

keep_speaker_busy: some speaker may go into low power mode, which may result in truncated speech beginnings. Set to true to play low frequency, low volume noise.
