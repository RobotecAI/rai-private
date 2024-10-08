# Human Robot Interface via Voice

> [!IMPORTANT]
> RAI_ASR supports both local Whisper models and OpenAI Whisper (cloud). When using the cloud version, the OPENAI_API_KEY environment variable must be set with a valid API key.

## Running example

When your robot's whoami package is ready, run the following:

> [!TIP]
> Make sure rai_whoami is running.

** Parameters **
recording_device: The device you want to record with. Check available with:

```bash
python -c 'import sounddevice as sd; print(sd.query_devices())'
```

keep_speaker_busy: some speakers may go into low power mode, which may result in truncated speech beginnings. Set to true to play low frequency, low volume noise to prevent sleep mode.

### OpenTTS

```bash
ros2 launch rai_bringup hri.launch.py tts_vendor:=opentts robot_description_package:=<robot_description_package> recording_device:=0 keep_speaker_busy:=(true|false) asr_vendor:=(whisper|openai)

```

> [!NOTE]
> Run OpenTTS with `docker run -it -p 5500:5500 synesthesiam/opentts:en --no-espeak`

### ElevenLabs

```bash
ros2 launch rai_bringup hri.launch.py robot_description_package:=<robot_description_package> recording_device:=0 keep_speaker_busy:=(true|false) asr_vendor:=(whisper|openai)
```
