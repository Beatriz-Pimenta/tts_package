# Package Information

How it works:
  - The interaction is based on a publisher/subscriber pattern, in which publisher_node.py publishes a message on the /tts_input topic and tts_node.py receives it, speaks the text, and saves the audio file related to that text.
  - The audio files are stored in the .MP3 format in the following directory: install/tts_package/share/tts_package/audio. It will create a folder called "audio" if you don't already have one.

How to run it:
  - To launch everything, use: ros2 launch tts_package tts_launch.py
  - To only run the publisher, use: ros2 run tts_package tts_pubisher
  - To only run the TTS node, use: ros2 run tts_package tts_node

This package contains:
  - TTS Node ---> tts_node.py
  - Publisher Node ---> publisher_node.py
  - Launch File ---> tts_launch.py
