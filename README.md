# tts_package

This project implements a text-to-speech (TTS) system using ROS 2, based on a publisher/subscriber communication pattern. The publisher_node.py sends text messages to the /tts_input topic, which tts_node.py receives to synthesize speech and save audio files in MP3 format. It automates audio generation and storage, making it useful for robotic applications or systems requiring dynamic speech output.

---

## Table of Contents

- [Package Information](#package-information)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Setup](#setup)
- [Usage](#usage)

---

## Package Information

### How it works:
- The interaction is based on a publisher/subscriber pattern, where `publisher_node.py` publishes a message on the `/tts_input` topic.
- `tts_node.py` subscribes to this topic, speaks the text, and saves the corresponding audio file.
- Audio files are saved in `.MP3` format inside the directory:  
  `install/tts_package/share/tts_package/audio`.  
  If this folder doesn’t exist, it will be created automatically.

### How to run it:
- To launch everything:  
  ``` bash
  ros2 launch tts_package tts_launch.py
  ```
- To run only the publisher:  
  ``` bash
  ros2 run tts_package tts_publisher
  ```
- To run only the TTS node:  
  ``` bash
  ros2 run tts_package tts_node
  ```

---

## Prerequisites

- Before installing, ensure you have the following installed on your system:
  - ROS2 Humble
  - libespeak-dev (for text-to-speech support)
  - pulseaudio (audio system for sound output)

- On Ubuntu, you can install these dependencies with:
```
sudo apt update
sudo apt install -y libespeak-dev pulseaudio
```

---

## Installation
``` bash
# Clone the repository
git clone https://github.com/Beatriz-Pimenta/tts_package.git

# Change directory
cd tts_package

# Install ROS 2 dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Install system dependencies (if not installed)
sudo apt install -y libespeak-dev pulseaudio

# Install Python dependencies
pip install -r requirements.txt

# Build the package (run from your ROS 2 workspace root)
colcon build --packages-select tts_package

# Source the workspace
source install/setup.bash
```

---

## Setup
- Configure PulseAudio (optional but recommended)
  - PulseAudio should run as a user daemon. To start PulseAudio manually (if it’s not already running):
  ``` bash
  pulseaudio --start
  ```
  - If you encounter issues with PulseAudio in ROS nodes, you may need to ensure your user has access or run PulseAudio with appropriate permissions.

---

## Usage
``` bash
# Launch the entire TTS system (publisher + TTS node)
ros2 launch tts_package tts_launch.py

# Run only the publisher node
ros2 run tts_package tts_publisher

# Run only the TTS node
ros2 run tts_package tts_node
```