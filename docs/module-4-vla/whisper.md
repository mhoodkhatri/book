---
sidebar_position: 2
title: "Chapter 1: Speech to Action with Whisper"
description: "Integrate OpenAI Whisper for speech recognition in robotics. Enable voice-controlled robot interaction and command execution through real-time audio processing."
keywords: [Whisper, speech recognition, voice control, OpenAI, robotics, NLP, audio processing, ROS 2]
---

# Chapter 1: Speech to Action with Whisper

**Giving Your Robot Ears**

---

## Chapter Overview

Imagine standing in your kitchen and saying, "Robot, bring me a glass of water." The robot hears you, understands the command, and rolls off to complete the task. This interaction feels natural—it is how we communicate with each other. But making it work requires solving a hard problem: converting the sound waves of your voice into text that a computer can process.

This chapter teaches you to integrate **OpenAI Whisper**, a state-of-the-art speech recognition model, into your robotics pipeline. By the end, your robot will be able to hear spoken commands and convert them into actionable text.

### What You Will Learn

- How speech recognition works at a conceptual level
- The Whisper model architecture and its capabilities
- How to process audio in real-time for robotics applications
- Building a ROS 2 node that listens, transcribes, and publishes commands
- Handling the challenges of noisy environments and diverse accents

### Prerequisites

Before starting this chapter, ensure you have:

- [ ] Completed the Module 4 introduction
- [ ] Python 3.10+ with PyTorch installed
- [ ] A working ROS 2 Humble environment
- [ ] (Optional) A USB microphone for live testing

---

## The Problem: From Sound to Meaning

When you speak, you create pressure waves in the air. These waves travel to a microphone, which converts them into electrical signals. Those signals are digitized into numbers—thousands of samples per second. The challenge is turning those numbers back into words.

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    SPEECH RECOGNITION PIPELINE                          │
│                                                                          │
│   Sound waves → Microphone → Digital samples → Model → Text             │
│                                                                          │
│   "Hello robot"                                                          │
│        │                                                                 │
│        ▼                                                                 │
│   [Pressure waves at ~344 m/s]                                          │
│        │                                                                 │
│        ▼                                                                 │
│   [Electrical signal]                                                    │
│        │                                                                 │
│        ▼                                                                 │
│   [16000 samples/second, each -1.0 to 1.0]                              │
│        │                                                                 │
│        ▼                                                                 │
│   [Whisper model]                                                        │
│        │                                                                 │
│        ▼                                                                 │
│   "Hello robot"                                                          │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

This is not trivial. Human speech is:

- **Continuous**: Words run together without clear boundaries
- **Variable**: Different speakers, accents, speeds, pitches
- **Noisy**: Background sounds, echoes, microphone quality
- **Contextual**: "I scream" sounds like "ice cream"

Traditional speech recognition systems used complex pipelines with acoustic models, language models, and pronunciation dictionaries. Modern systems like Whisper use end-to-end deep learning—one neural network that learns everything from data.

---

## Introducing Whisper

**Whisper** is an automatic speech recognition (ASR) system released by OpenAI. It was trained on 680,000 hours of audio data covering multiple languages and tasks. The key insight behind Whisper is scale: train on enough diverse data, and the model learns to handle the messy reality of human speech.

### Whisper Architecture

Whisper uses an **encoder-decoder transformer** architecture:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         WHISPER ARCHITECTURE                             │
│                                                                          │
│                        Audio Input (30s max)                             │
│                              │                                           │
│                              ▼                                           │
│                    ┌─────────────────┐                                   │
│                    │   Mel Spectrogram│  Convert audio to image-like     │
│                    │   (80 x 3000)    │  representation                  │
│                    └────────┬────────┘                                   │
│                             │                                            │
│                             ▼                                            │
│                    ┌─────────────────┐                                   │
│                    │     ENCODER     │  Understand the audio             │
│                    │  (Transformer)  │                                   │
│                    └────────┬────────┘                                   │
│                             │                                            │
│                             ▼                                            │
│                    ┌─────────────────┐                                   │
│                    │     DECODER     │  Generate text tokens             │
│                    │  (Transformer)  │  one at a time                    │
│                    └────────┬────────┘                                   │
│                             │                                            │
│                             ▼                                            │
│                      Text Output                                         │
│                    "Hello robot"                                         │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

**Mel Spectrogram**: The audio waveform is converted into a mel spectrogram—a visual representation of frequencies over time. Think of it as turning sound into an image that the neural network can "see."

**Encoder**: The transformer encoder processes the spectrogram and creates a rich representation of the audio content.

**Decoder**: The transformer decoder generates text tokens one at a time, using the encoder output and previously generated tokens.

### Whisper Model Sizes

Whisper comes in multiple sizes. Larger models are more accurate but slower:

| Model | Parameters | VRAM Required | Relative Speed | English Word Error Rate |
|-------|------------|---------------|----------------|------------------------|
| tiny | 39M | ~1 GB | ~32x | 7.6% |
| base | 74M | ~1 GB | ~16x | 5.0% |
| small | 244M | ~2 GB | ~6x | 3.4% |
| medium | 769M | ~5 GB | ~2x | 2.9% |
| large | 1550M | ~10 GB | 1x | 2.7% |

For robotics, we often use **small** or **medium** as a balance between accuracy and speed. The **tiny** model is useful for resource-constrained edge devices.

---

## Setting Up Whisper

Let us install Whisper and verify it works before integrating with ROS 2.

### Installation

```bash
# Create a virtual environment (recommended)
cd ~/ros2_ws/src
python3 -m venv whisper_env
source whisper_env/bin/activate

# Install Whisper and dependencies
pip install openai-whisper
pip install torch torchaudio

# For faster inference on NVIDIA GPUs (optional but recommended)
pip install faster-whisper
```

### Basic Usage

Create a test script to verify installation:

```python
#!/usr/bin/env python3
"""
whisper_test.py
Basic Whisper transcription test.

This script demonstrates the simplest possible Whisper usage:
load a model and transcribe an audio file.
"""

import whisper

def main():
    # Load the model (downloads on first run)
    # Use "tiny" for fast testing, "small" or "medium" for production
    print("Loading Whisper model...")
    model = whisper.load_model("small")
    print("Model loaded successfully.")

    # Transcribe an audio file
    # Replace with your own audio file path
    audio_path = "test_audio.wav"

    print(f"Transcribing: {audio_path}")
    result = model.transcribe(audio_path)

    # Print results
    print("\n=== Transcription Result ===")
    print(f"Text: {result['text']}")
    print(f"Language: {result['language']}")

    # Show segments with timestamps
    print("\n=== Segments ===")
    for segment in result['segments']:
        start = segment['start']
        end = segment['end']
        text = segment['text']
        print(f"[{start:.2f}s - {end:.2f}s]: {text}")

if __name__ == "__main__":
    main()
```

### Creating Test Audio

If you do not have an audio file, create one with your microphone:

```bash
# Record 5 seconds of audio (requires arecord on Linux)
arecord -d 5 -f cd -t wav test_audio.wav

# Or use Python to record
pip install sounddevice scipy
```

```python
#!/usr/bin/env python3
"""
record_audio.py
Record audio from the default microphone.
"""

import sounddevice as sd
from scipy.io.wavfile import write

# Recording parameters
SAMPLE_RATE = 16000  # Whisper expects 16kHz
DURATION = 5  # seconds

print(f"Recording for {DURATION} seconds...")
print("Speak now!")

# Record audio
audio = sd.rec(
    int(DURATION * SAMPLE_RATE),
    samplerate=SAMPLE_RATE,
    channels=1,
    dtype='float32'
)
sd.wait()  # Wait until recording is finished

# Save to file
write("test_audio.wav", SAMPLE_RATE, audio)
print("Saved to test_audio.wav")
```

Run the test:

```bash
# Record audio
python3 record_audio.py

# Transcribe
python3 whisper_test.py
```

Expected output:

```
Loading Whisper model...
Model loaded successfully.
Transcribing: test_audio.wav

=== Transcription Result ===
Text:  Hello robot, please go to the kitchen.
Language: en

=== Segments ===
[0.00s - 2.50s]:  Hello robot, please go to the kitchen.
```

---

## Real-Time Audio Processing

For a robot to respond to voice commands, it cannot wait for you to finish speaking and save a file. It needs to process audio in **real-time**. This introduces new challenges.

### The Streaming Problem

Whisper was designed for batch processing—give it a complete audio file, get back text. For real-time use, we need to:

1. **Continuously capture audio** from the microphone
2. **Buffer audio** into chunks (Whisper processes up to 30 seconds)
3. **Detect when speech ends** (voice activity detection)
4. **Transcribe completed utterances** as quickly as possible
5. **Handle the latency** between speaking and getting results

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    REAL-TIME PROCESSING PIPELINE                         │
│                                                                          │
│   Microphone                                                             │
│       │                                                                  │
│       ▼                                                                  │
│   ┌─────────────────┐                                                    │
│   │  Audio Buffer   │  Accumulate samples                                │
│   │   (16kHz)       │                                                    │
│   └────────┬────────┘                                                    │
│            │                                                             │
│            ▼                                                             │
│   ┌─────────────────┐                                                    │
│   │  Voice Activity │  Detect speech vs silence                          │
│   │   Detection     │                                                    │
│   └────────┬────────┘                                                    │
│            │                                                             │
│      ┌─────┴─────┐                                                       │
│      │           │                                                       │
│   Silence     Speech                                                     │
│      │           │                                                       │
│      ▼           ▼                                                       │
│   Continue   ┌─────────────────┐                                         │
│   buffering  │  Send to        │                                         │
│              │  Whisper        │                                         │
│              └────────┬────────┘                                         │
│                       │                                                  │
│                       ▼                                                  │
│              ┌─────────────────┐                                         │
│              │  Transcription  │                                         │
│              │  Result         │                                         │
│              └─────────────────┘                                         │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### Implementing Real-Time Transcription

Let us build a production-quality real-time transcription system step by step. We will use `faster-whisper` for GPU acceleration and `webrtcvad` for voice activity detection.

First, install the required dependencies:

```bash
pip install faster-whisper webrtcvad sounddevice numpy
```

Now let us build the system piece by piece, understanding each component.

#### Step 1: Import Required Libraries

```python
#!/usr/bin/env python3
"""
realtime_whisper.py
Real-time speech recognition using Whisper with voice activity detection.
"""

import numpy as np          # For audio data manipulation
import sounddevice as sd    # For microphone access
import queue               # Thread-safe queue for audio data
import threading           # For running audio capture in background
from faster_whisper import WhisperModel  # Optimized Whisper
import webrtcvad           # Voice Activity Detection
```

**What each library does:**
- `numpy`: Handles audio data as arrays of numbers
- `sounddevice`: Captures audio from your microphone
- `queue`: Safely passes audio between threads
- `faster_whisper`: A faster implementation of Whisper
- `webrtcvad`: Detects when someone is speaking vs. silence

#### Step 2: Initialize the Transcriber

```python
class RealtimeTranscriber:
    """
    Handles real-time audio capture and transcription.

    Two things happen simultaneously:
    1. Audio capture: Continuously reads from microphone
    2. Processing: Analyzes audio and transcribes when speech ends
    """

    def __init__(
        self,
        model_size: str = "small",
        sample_rate: int = 16000,
        vad_aggressiveness: int = 2
    ):
        # Store settings
        self.sample_rate = sample_rate  # 16000 Hz is what Whisper expects
        self.audio_queue = queue.Queue()  # Audio data waits here
        self.running = False  # Controls the main loop
```

**Key parameters explained:**
- `model_size`: "tiny" is fast but less accurate, "large" is slow but very accurate
- `sample_rate`: 16000 means 16,000 audio samples per second (Whisper's requirement)
- `vad_aggressiveness`: 0-3, higher values ignore more background noise

#### Step 3: Load the AI Models

```python
        # Load Whisper model (this downloads the model on first run)
        print(f"Loading Whisper {model_size} model...")
        self.model = WhisperModel(
            model_size,
            device="cuda",      # Use GPU (change to "cpu" if no GPU)
            compute_type="float16"  # float16 is faster, int8 even faster
        )
        print("Model loaded.")

        # Initialize Voice Activity Detection
        # VAD tells us when someone is speaking vs silence
        self.vad = webrtcvad.Vad(vad_aggressiveness)
```

**Why two models?**
- **Whisper**: Converts speech to text (expensive, slow)
- **VAD**: Detects if audio contains speech (cheap, fast)

We use VAD to know *when* to run Whisper, saving computation.

#### Step 4: Set Up Speech Buffering

```python
        # Buffer for accumulating speech
        self.speech_buffer = []  # Stores audio frames while someone speaks
        self.silence_frames = 0  # Counts consecutive silent frames
        self.speech_frames = 0   # Counts frames with speech

        # Timing thresholds (each frame is 30 milliseconds)
        self.min_speech_frames = 10   # Minimum ~300ms of speech
        self.max_silence_frames = 30  # End after ~900ms of silence
```

**Why buffer?**
- We cannot send every millisecond to Whisper (too slow)
- We wait until a complete sentence/phrase is spoken
- When we detect ~1 second of silence, we know the person stopped talking

#### Step 5: Handle Incoming Audio

```python
    def audio_callback(self, indata, frames, time, status):
        """
        Called automatically whenever new audio arrives from microphone.
        This runs in a separate thread managed by sounddevice.
        """
        if status:
            print(f"Audio warning: {status}")

        # Put the audio in our queue for processing
        # We copy it because the original buffer gets reused
        self.audio_queue.put(indata.copy())
```

**How audio capture works:**
1. `sounddevice` runs in the background
2. Every 30ms, it gives us a chunk of audio
3. We put it in a queue (like a waiting line)
4. Our main loop processes items from the queue

#### Step 6: Process Audio and Detect Speech

```python
    def process_audio(self):
        """
        Take audio from the queue and check if it contains speech.
        Returns transcription when a complete utterance is detected.
        """
        # Get the next audio chunk (wait up to 0.1 seconds)
        try:
            audio_block = self.audio_queue.get(timeout=0.1)
        except queue.Empty:
            return None  # No audio available

        # Convert audio format for VAD
        # VAD needs 16-bit integers, but sounddevice gives us floats
        audio_int16 = (audio_block * 32767).astype(np.int16)

        # Process in 30ms frames (VAD requirement)
        frame_duration_ms = 30
        frame_size = int(self.sample_rate * frame_duration_ms / 1000)
        # At 16kHz, 30ms = 480 samples
```

**Audio format conversion:**
- Microphone gives us: floating point numbers (-1.0 to 1.0)
- VAD needs: 16-bit integers (-32767 to 32767)
- We multiply by 32767 to convert

#### Step 7: Voice Activity Detection Logic

```python
        # Check each 30ms frame for speech
        for i in range(0, len(audio_int16), frame_size):
            frame = audio_int16[i:i + frame_size]

            # Pad if frame is too short
            if len(frame) < frame_size:
                frame = np.pad(frame, (0, frame_size - len(frame)))

            # Ask VAD: "Does this frame contain speech?"
            is_speech = self.vad.is_speech(frame.tobytes(), self.sample_rate)

            if is_speech:
                # Someone is speaking!
                self.speech_buffer.append(frame)
                self.speech_frames += 1
                self.silence_frames = 0  # Reset silence counter
            else:
                # Silence detected
                if self.speech_frames > 0:
                    # We were recording speech, now tracking silence
                    self.silence_frames += 1
                    self.speech_buffer.append(frame)  # Keep some trailing silence

                    # Is the utterance complete?
                    if self.silence_frames >= self.max_silence_frames:
                        if self.speech_frames >= self.min_speech_frames:
                            # Yes! Transcribe what we captured
                            return self._transcribe_buffer()
                        else:
                            # Too short - probably just noise
                            self._reset_buffer()

        return None  # Still listening...
```

**The detection logic visualized:**
```
Audio:    [silence][HELLO ROBOT][silence...silence...silence]
VAD:      [  no   ][ yes  yes ][ no   no   no   no   no  ]
Action:            [--buffering--][wait 30 frames][TRANSCRIBE!]
```

#### Step 8: Transcribe the Buffered Audio

```python
    def _transcribe_buffer(self) -> str:
        """Convert the buffered audio to text using Whisper."""
        if not self.speech_buffer:
            return ""

        # Combine all the frames into one audio clip
        audio_data = np.concatenate(self.speech_buffer)

        # Convert back to float32 (Whisper's expected format)
        audio_float = audio_data.astype(np.float32) / 32767.0

        # Run Whisper!
        segments, info = self.model.transcribe(
            audio_float,
            beam_size=5,        # Higher = more accurate but slower
            language="en",      # Set to None for auto-detection
            vad_filter=True     # Extra filtering for cleaner results
        )

        # Combine all text segments
        transcription = " ".join(segment.text for segment in segments)

        # Clean up for next utterance
        self._reset_buffer()

        return transcription.strip()

    def _reset_buffer(self):
        """Clear the buffer to start fresh."""
        self.speech_buffer = []
        self.speech_frames = 0
        self.silence_frames = 0
```

**Whisper parameters explained:**
- `beam_size=5`: Considers 5 possible transcriptions, picks the best
- `language="en"`: Assumes English (faster than auto-detect)
- `vad_filter=True`: Additional cleaning of the transcription

#### Step 9: The Main Listening Loop

```python
    def start(self, callback=None):
        """
        Begin listening for speech.

        Args:
            callback: Function to call when speech is transcribed
        """
        self.running = True

        # Open the microphone
        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,          # Mono audio (one channel)
            dtype='float32',     # Audio format
            blocksize=int(self.sample_rate * 0.03),  # 30ms chunks
            callback=self.audio_callback  # Called for each chunk
        ):
            print("Listening... (Ctrl+C to stop)")

            # Main processing loop
            while self.running:
                transcription = self.process_audio()

                if transcription:
                    print(f"\nTranscribed: {transcription}")
                    if callback:
                        callback(transcription)

    def stop(self):
        """Stop the listening loop."""
        self.running = False
```

#### Step 10: Putting It All Together

```python
def main():
    """Run the real-time transcription demo."""
    # Create transcriber with small model (good balance)
    transcriber = RealtimeTranscriber(model_size="small")

    def on_transcription(text):
        """Called whenever speech is transcribed."""
        print(f"[COMMAND RECEIVED]: {text}")
        # In a real robot, you would process this command here

    try:
        transcriber.start(callback=on_transcription)
    except KeyboardInterrupt:
        print("\nStopping...")
        transcriber.stop()


if __name__ == "__main__":
    main()
```

**Running the demo:**
```bash
python3 realtime_whisper.py

# Output:
# Loading Whisper small model...
# Model loaded.
# Listening... (Ctrl+C to stop)
#
# Transcribed:  Hello robot go to the kitchen
# [COMMAND RECEIVED]:  Hello robot go to the kitchen
```

---

## Integrating with ROS 2

Now let us wrap our speech recognition in a ROS 2 node so it can communicate with the rest of the robot system. We will build this step by step.

### Understanding the ROS 2 Speech Node

Our node will:
1. Listen to audio from the microphone
2. Transcribe speech using Whisper
3. Publish transcriptions to ROS 2 topics
4. Filter for robot-specific commands

```
┌─────────────────────────────────────────────────────────────────────┐
│                      WHISPER ROS 2 NODE                              │
│                                                                      │
│   Microphone ───▶ [Whisper Node] ───▶ /speech/transcription         │
│                          │                                           │
│                          └──────────▶ /speech/command                │
│                                       (filtered for robot commands)  │
└─────────────────────────────────────────────────────────────────────┘
```

### Step 1: Node Setup and Imports

```python
#!/usr/bin/env python3
"""
whisper_node.py
ROS 2 node for speech recognition using Whisper.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import numpy as np
import sounddevice as sd
import queue
import threading
from faster_whisper import WhisperModel
import webrtcvad
```

### Step 2: Initialize the Node with Parameters

ROS 2 parameters allow you to configure the node without changing code.

```python
class WhisperNode(Node):
    """ROS 2 node that provides speech recognition capabilities."""

    def __init__(self):
        super().__init__('whisper_node')

        # Declare configurable parameters
        self.declare_parameter('model_size', 'small')   # Whisper model
        self.declare_parameter('sample_rate', 16000)    # Audio sample rate
        self.declare_parameter('language', 'en')        # Expected language
        self.declare_parameter('vad_aggressiveness', 2) # Noise filtering
        self.declare_parameter('device', 'cuda')        # GPU or CPU

        # Read the parameter values
        model_size = self.get_parameter('model_size').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.language = self.get_parameter('language').value
        vad_aggressiveness = self.get_parameter('vad_aggressiveness').value
        device = self.get_parameter('device').value
```

**Why parameters?** They let you change settings at launch time:
```bash
ros2 run whisper_ros whisper_node --ros-args -p model_size:=tiny -p device:=cpu
```

### Step 3: Set Up Publishers

Publishers send data to other ROS 2 nodes.

```python
        # Publisher for ALL transcribed speech
        self.transcription_pub = self.create_publisher(
            String,                    # Message type
            'speech/transcription',    # Topic name
            10                         # Queue size
        )

        # Publisher for ROBOT COMMANDS only
        self.command_pub = self.create_publisher(
            String,
            'speech/command',
            10
        )
```

**Two topics because:**
- `/speech/transcription`: Everything the user says (for logging, debugging)
- `/speech/command`: Only speech that looks like robot commands

### Step 4: Initialize Audio and AI Components

```python
        # Queue for passing audio between threads
        self.audio_queue = queue.Queue()

        # Load Whisper model
        self.get_logger().info(f'Loading Whisper {model_size} model...')
        self.model = WhisperModel(
            model_size,
            device=device,
            compute_type="float16" if device == "cuda" else "int8"
        )
        self.get_logger().info('Model loaded successfully.')

        # Initialize Voice Activity Detection
        self.vad = webrtcvad.Vad(vad_aggressiveness)

        # Speech buffer state (same as standalone version)
        self.speech_buffer = []
        self.silence_frames = 0
        self.speech_frames = 0
        self.min_speech_frames = 10
        self.max_silence_frames = 30
```

### Step 5: Define Robot Command Keywords

We filter transcriptions to find robot-relevant commands.

```python
        # Keywords that indicate a robot command
        self.command_keywords = [
            # Movement
            'go', 'move', 'navigate', 'drive',
            'stop', 'halt', 'pause',
            'turn', 'rotate', 'spin',
            # Manipulation
            'pick', 'grab', 'take',
            'place', 'put', 'drop',
            # Perception
            'look', 'find', 'search',
            'follow', 'come', 'bring'
        ]
```

**Why filter?** We do not want the robot reacting to "I'm going to the store." Only "Robot, go to the kitchen."

### Step 6: Start the Audio Thread

ROS 2 needs its main thread for communication, so audio runs separately.

```python
        # Start audio processing in background
        self.running = True
        self.audio_thread = threading.Thread(target=self._audio_loop)
        self.audio_thread.daemon = True  # Thread dies when main program exits
        self.audio_thread.start()

        self.get_logger().info('Whisper node started. Listening for speech...')
```

### Step 7: Audio Capture Loop

```python
    def _audio_callback(self, indata, frames, time, status):
        """Called when new audio arrives from microphone."""
        if status:
            self.get_logger().warn(f'Audio status: {status}')
        self.audio_queue.put(indata.copy())

    def _audio_loop(self):
        """Runs in background thread - captures and processes audio."""
        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            dtype='float32',
            blocksize=int(self.sample_rate * 0.03),  # 30ms blocks
            callback=self._audio_callback
        ):
            while self.running:
                self._process_audio()
```

### Step 8: Process Audio (Same Logic as Before)

```python
    def _process_audio(self):
        """Process audio from queue, detect speech, trigger transcription."""
        try:
            audio_block = self.audio_queue.get(timeout=0.1)
        except queue.Empty:
            return

        # Convert to int16 for VAD
        audio_int16 = (audio_block * 32767).astype(np.int16)
        frame_duration_ms = 30
        frame_size = int(self.sample_rate * frame_duration_ms / 1000)

        for i in range(0, len(audio_int16), frame_size):
            frame = audio_int16[i:i + frame_size]
            if len(frame) < frame_size:
                frame = np.pad(frame, (0, frame_size - len(frame)))

            is_speech = self.vad.is_speech(frame.tobytes(), self.sample_rate)

            if is_speech:
                self.speech_buffer.append(frame)
                self.speech_frames += 1
                self.silence_frames = 0
            else:
                if self.speech_frames > 0:
                    self.silence_frames += 1
                    self.speech_buffer.append(frame)

                    if self.silence_frames >= self.max_silence_frames:
                        if self.speech_frames >= self.min_speech_frames:
                            self._transcribe_and_publish()
                        else:
                            self._reset_buffer()
```

### Step 9: Transcribe and Publish to ROS 2 Topics

This is where ROS 2 integration happens.

```python
    def _transcribe_and_publish(self):
        """Transcribe buffered audio and publish to ROS topics."""
        if not self.speech_buffer:
            return

        # Prepare audio for Whisper
        audio_data = np.concatenate(self.speech_buffer)
        audio_float = audio_data.astype(np.float32) / 32767.0

        # Run transcription
        segments, _ = self.model.transcribe(
            audio_float,
            beam_size=5,
            language=self.language if self.language != 'auto' else None,
            vad_filter=True
        )
        transcription = " ".join(segment.text for segment in segments).strip()

        if transcription:
            # Publish to transcription topic (all speech)
            msg = String()
            msg.data = transcription
            self.transcription_pub.publish(msg)
            self.get_logger().info(f'Transcribed: "{transcription}"')

            # Check if it's a robot command and publish if so
            command = self._extract_command(transcription)
            if command:
                cmd_msg = String()
                cmd_msg.data = command
                self.command_pub.publish(cmd_msg)
                self.get_logger().info(f'Command: "{command}"')

        self._reset_buffer()
```

### Step 10: Command Extraction

```python
    def _extract_command(self, text: str) -> str:
        """
        Check if transcription contains a robot command.
        Returns the text if it contains a command keyword, empty string otherwise.
        """
        text_lower = text.lower()

        for keyword in self.command_keywords:
            if keyword in text_lower:
                return text  # It's a command!

        return ""  # Not a command

    def _reset_buffer(self):
        """Clear speech buffer for next utterance."""
        self.speech_buffer = []
        self.speech_frames = 0
        self.silence_frames = 0
```

### Step 11: Clean Shutdown

```python
    def destroy_node(self):
        """Clean up when node shuts down."""
        self.running = False
        if self.audio_thread.is_alive():
            self.audio_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WhisperNode()

    try:
        rclpy.spin(node)  # Keep node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Package Setup

Create the ROS 2 package structure:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python whisper_ros \
    --dependencies rclpy std_msgs
```

Add the node to `setup.py`:

```python
from setuptools import setup

package_name = 'whisper_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/whisper.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='ROS 2 speech recognition using Whisper',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'whisper_node = whisper_ros.whisper_node:main',
        ],
    },
)
```

### Launch File

Create `launch/whisper.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'model_size',
            default_value='small',
            description='Whisper model size (tiny, base, small, medium, large)'
        ),
        DeclareLaunchArgument(
            'language',
            default_value='en',
            description='Language code or "auto" for detection'
        ),
        DeclareLaunchArgument(
            'device',
            default_value='cuda',
            description='Compute device (cuda or cpu)'
        ),

        # Whisper node
        Node(
            package='whisper_ros',
            executable='whisper_node',
            name='whisper_node',
            parameters=[{
                'model_size': LaunchConfiguration('model_size'),
                'language': LaunchConfiguration('language'),
                'device': LaunchConfiguration('device'),
            }],
            output='screen'
        ),
    ])
```

### Build and Run

```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select whisper_ros
source install/setup.bash

# Run the node
ros2 launch whisper_ros whisper.launch.py

# In another terminal, listen to transcriptions
ros2 topic echo /speech/transcription

# And commands
ros2 topic echo /speech/command
```

---

## Handling Real-World Challenges

Speech recognition in robotics faces challenges that do not exist in controlled environments.

### Noisy Environments

Robots operate in noisy places—factories, kitchens, outdoor spaces. Background noise degrades recognition accuracy.

**Strategies:**

1. **Better Microphones**: Directional microphones focus on the speaker
2. **Noise Reduction**: Pre-process audio to remove background noise
3. **Keyword Spotting**: Listen for wake words before full transcription
4. **Larger Models**: Bigger Whisper models handle noise better

```python
# Add noise reduction with noisereduce library
import noisereduce as nr

def reduce_noise(audio, sample_rate):
    """Apply noise reduction to audio."""
    return nr.reduce_noise(
        y=audio,
        sr=sample_rate,
        stationary=True,  # Assume stationary background noise
        prop_decrease=0.75  # How much to reduce noise
    )
```

### Multiple Languages

Whisper supports 99 languages. For multilingual environments:

```python
# Auto-detect language
segments, info = model.transcribe(
    audio,
    language=None  # Auto-detect
)
print(f"Detected language: {info.language}")

# Or specify expected languages
segments, info = model.transcribe(
    audio,
    language=None,
    task="transcribe"  # or "translate" to translate to English
)
```

### Wake Words

You do not want your robot transcribing every conversation in the room. Implement a wake word system:

```python
class WakeWordDetector:
    """
    Simple wake word detection using Whisper.

    Listens for a specific phrase before activating full transcription.
    """

    def __init__(self, wake_phrase="hey robot"):
        self.wake_phrase = wake_phrase.lower()
        self.is_active = False
        self.timeout_seconds = 10  # How long to stay active
        self.last_wake_time = 0

    def check_wake_word(self, transcription: str) -> bool:
        """Check if transcription contains wake word."""
        if self.wake_phrase in transcription.lower():
            self.is_active = True
            self.last_wake_time = time.time()
            return True
        return False

    def is_listening(self) -> bool:
        """Check if we should process commands."""
        if not self.is_active:
            return False

        # Check timeout
        if time.time() - self.last_wake_time > self.timeout_seconds:
            self.is_active = False
            return False

        return True
```

---

## Command Mapping

Transcribed text is just text. We need to map it to robot actions. This is a bridge to Chapter 2 where we will use LLMs for sophisticated understanding.

### Simple Keyword Mapping

For basic commands, pattern matching works:

```python
import re

class CommandMapper:
    """
    Map transcribed speech to robot commands.

    This is a simple rule-based approach. Chapter 2 covers
    LLM-based understanding for complex commands.
    """

    def __init__(self):
        # Pattern -> (action, parameter_extractor)
        self.patterns = [
            # Navigation commands
            (r"go to (?:the )?(.+)", "navigate", lambda m: {"location": m.group(1)}),
            (r"move (?:forward|ahead)(?: (\d+))?", "move_forward",
             lambda m: {"distance": float(m.group(1) or 1)}),
            (r"turn (left|right)(?: (\d+))?", "turn",
             lambda m: {"direction": m.group(1), "degrees": float(m.group(2) or 90)}),

            # Object interaction
            (r"pick up (?:the )?(.+)", "pick", lambda m: {"object": m.group(1)}),
            (r"put down (?:the )?(.+)", "place", lambda m: {"object": m.group(1)}),

            # Control commands
            (r"stop", "stop", lambda m: {}),
            (r"cancel", "cancel", lambda m: {}),
        ]

    def parse(self, text: str) -> dict:
        """
        Parse transcribed text into a robot command.

        Returns:
            dict with 'action' and 'parameters' keys,
            or None if no command matched
        """
        text = text.lower().strip()

        for pattern, action, extractor in self.patterns:
            match = re.search(pattern, text)
            if match:
                return {
                    "action": action,
                    "parameters": extractor(match),
                    "raw_text": text
                }

        return None


# Usage
mapper = CommandMapper()

commands = [
    "Go to the kitchen",
    "Turn left 45 degrees",
    "Pick up the red cup",
    "Stop",
]

for text in commands:
    result = mapper.parse(text)
    print(f"'{text}' -> {result}")
```

Output:

```
'Go to the kitchen' -> {'action': 'navigate', 'parameters': {'location': 'kitchen'}, 'raw_text': 'go to the kitchen'}
'Turn left 45 degrees' -> {'action': 'turn', 'parameters': {'direction': 'left', 'degrees': 45.0}, 'raw_text': 'turn left 45 degrees'}
'Pick up the red cup' -> {'action': 'pick', 'parameters': {'object': 'red cup'}, 'raw_text': 'pick up the red cup'}
'Stop' -> {'action': 'stop', 'parameters': {}, 'raw_text': 'stop'}
```

---

## Hands-on Exercise

Build a complete voice-controlled robot command system.

### Exercise 1: Basic Setup

1. Install Whisper and dependencies
2. Record a test audio file
3. Verify transcription works

```bash
# Expected test
python3 -c "import whisper; m = whisper.load_model('tiny'); print('Success!')"
```

### Exercise 2: ROS 2 Integration

1. Create the `whisper_ros` package
2. Run the Whisper node
3. Verify topics are publishing

```bash
# Check topics exist
ros2 topic list | grep speech
# Expected: /speech/transcription, /speech/command

# Echo transcriptions while speaking
ros2 topic echo /speech/transcription
```

### Exercise 3: Command Processing

1. Implement the `CommandMapper` class
2. Add custom commands for your robot
3. Test with various phrasings

```python
# Test your mapper handles variations
test_phrases = [
    "go to kitchen",
    "Go to the kitchen",
    "please go to the kitchen",
    "can you go to the kitchen",
]
# All should map to navigate->kitchen
```

### Exercise 4: Integration Test

1. Connect Whisper output to a simulated robot
2. Speak commands and watch the robot respond
3. Measure latency from speech to action

---

## Performance Optimization

For production robotics, every millisecond counts.

### GPU Acceleration

Ensure you are using GPU:

```python
# Check CUDA availability
import torch
print(f"CUDA available: {torch.cuda.is_available()}")
print(f"GPU: {torch.cuda.get_device_name(0)}")
```

### Model Quantization

Use quantized models for faster inference:

```python
# Use int8 quantization
model = WhisperModel(
    "small",
    device="cuda",
    compute_type="int8"  # Faster than float16
)
```

### Batch Processing

If processing pre-recorded audio, batch for efficiency:

```python
# Process multiple files in parallel
from concurrent.futures import ThreadPoolExecutor

def transcribe_file(path):
    return model.transcribe(path)

files = ["audio1.wav", "audio2.wav", "audio3.wav"]

with ThreadPoolExecutor(max_workers=2) as executor:
    results = list(executor.map(transcribe_file, files))
```

---

## Common Issues and Solutions

### Issue: "CUDA out of memory"

**Solution**: Use a smaller model or reduce batch size:

```python
# Use tiny model
model = WhisperModel("tiny", device="cuda")

# Or use CPU
model = WhisperModel("small", device="cpu")
```

### Issue: Poor transcription accuracy

**Solutions**:
1. Use a larger model
2. Improve microphone quality
3. Reduce background noise
4. Specify the language explicitly

### Issue: High latency

**Solutions**:
1. Use `faster-whisper` instead of base Whisper
2. Use smaller models (tiny, base)
3. Reduce VAD silence threshold
4. Use GPU acceleration

---

## Summary

In this chapter, you learned to give your robot the ability to hear and understand speech:

| Concept | What You Learned |
|---------|------------------|
| **Speech Recognition** | How audio becomes text |
| **Whisper** | State-of-the-art ASR model architecture |
| **Real-time Processing** | Voice activity detection and streaming |
| **ROS 2 Integration** | Publishing transcriptions as ROS topics |
| **Command Mapping** | Converting text to robot actions |

Your robot can now hear commands like "go to the kitchen" and convert them to text. But text alone is not enough—the robot needs to understand what you mean and plan how to achieve it.

---

## What's Next

In Chapter 2, you will learn to use Large Language Models for **cognitive planning**. Instead of simple keyword matching, your robot will understand complex instructions and break them into executable steps.

The journey continues: from hearing to understanding.

[Continue to Chapter 2: Cognitive Planning →](/docs/module-4-vla/cognitive-planning)

---

## Additional Resources

- [OpenAI Whisper GitHub](https://github.com/openai/whisper) - Official repository
- [Faster Whisper](https://github.com/guillaumekln/faster-whisper) - Optimized implementation
- [WebRTC VAD](https://github.com/wiseman/py-webrtcvad) - Voice activity detection
- [ROS 2 Audio Common](https://github.com/ros-drivers/audio_common) - Audio packages for ROS
- [Whisper Paper](https://cdn.openai.com/papers/whisper.pdf) - Technical details
