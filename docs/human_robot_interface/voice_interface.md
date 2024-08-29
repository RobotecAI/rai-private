# Human Robot Interface via Voice

> [!IMPORTANT]
> RAI_ASR is based on OpenAI Whisper model. It is expected to have OPENAI_API_KEY environment variable populated.

## Running example

When your robot's whoami package is ready, run the following:

> [!TIP]
> Make sure rai_whoami is running.

** Parameters **
recording_device: The device you want to record with. Check available with:

```bash
python -c 'import sounddevice as sd; print(sd.query_devices())'
```

keep_speaker_busy: some speaker may go into low power mode, which may result in truncated speech beginnings. Set to true to play low frequency, low volume noise.

### OpenTTS

```bash
ros2 launch rai_bringup hri.launch.py tts_vendor:=opentts robot_description_package:=<robot_description_package> recording_device:=0 keep_speaker_busy:=(true|false)

```

> [!NOTE]
> Run OpenTTS with `docker run -it -p 5500:5500 synesthesiam/opentts:en--no-espeak`

### ElevenLabs

```bash
ros2 launch rai_bringup hri.launch.py robot_description_package:=<robot_description_package> recording_device:=0 keep_speaker_busy:=(true|false)
```
