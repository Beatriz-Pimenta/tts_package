# this is the tts node
# it will create a subscriber to the /tts_input topic and speak the message it receives, 
# as well as save it to the "audio" folder
# (it creates one in the install/tts_package/share/tts_package directory of your workspace)
# it will also log every part of the process for a better understanding



# --- ROS 2 SYSTEM IMPORTS ---
import rclpy # ROS2 Python client library
from rclpy.node import Node # Base class for all ROS2 nodes
from std_msgs.msg import String # Message type (for text input)

import pyttsx3 # Text-to-Speech engines
import os # For file/folder handling
from datetime import datetime # For timestamped filenames

import multiprocessing
from ament_index_python.packages import get_package_share_directory # Helper to locate package directories
import threading # For background worker thread
import queue # Thread-safe queue for passing text messages
# ------------------------------------------------------------




# Runs in a separate process:
# (forces the engine to say every message, speaks the text and saves the audio file)
def speak_and_save(text, save_path):
    try:
        # Initialize the TTS engine
        engine = pyttsx3.init()

        # --- SET VOICE PARAMETERS RIGHT AWAY ---
        voices = engine.getProperty('voices')
        engine.setProperty('voice', voices[1].id) # change index to pick a voice (0 = male; 1 = female)
        engine.setProperty('rate', 180) # change the speech rate (words per minute)
        engine.setProperty('volume', 0.9) # change the volume (varies between 0.0 and 1.0)
        # ------------------------------------------

        # Play the speech
        engine.say(text)
        engine.runAndWait()

        # --- SAVE SPEECH TO A TIMESTAMPED .MP3 FILE ---
        filename = f"tts_{datetime.now():%Y%m%d_%H%M%S}.mp3"
        filepath = os.path.join(save_path, filename)

        engine.save_to_file(text, filepath)
        engine.runAndWait()
        # ------------------------------------------------

        # Log that the file was saved and where it was saved to
        print(f"[TTS WORKER] Spoken and saved: {filepath}")

    except Exception as e:
        # If pyttsx3 fails, this will print an error
        print(f"[TTS WORKER] ERROR: {e}")




# ROS2 Node Class:
class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node') # initialize the node with the name "tts_node"
        
        # Create a subscriber:
        self.subscription = self.create_subscription(
            String, # expected message type
            'tts_input', # listens to this topic
            self.listener_callback, # calls self.listener_callback when a message arrives
            10 # queue size --- max stored messages if callback is slow
        )

        # --- MESSAGE THREAD ---
        self.tts_queue = queue.Queue()
        # ------------------------------------------

        # --- SAVE FOLDER INSIDE PACKAGE SHARE DIRECTORY ---
        self.save_path = os.path.join(get_package_share_directory('tts_package'), 'audio') # save audio to: ros2_ws_new/src/tts_node/audio
        os.makedirs(self.save_path, exist_ok=True)
        # ---------------------------------------------------

        # Log that the node is working
        self.get_logger().info("TTS Node is running.")


    # Callback executed whenever a new String message arrives on /tts_input:
    def listener_callback(self, msg):
        text = msg.data # get the text from the message

        # Log the received text
        self.get_logger().info(f"Received text for TTS: '{text}'")

        # Spawn a new process for each TTS task
        p = multiprocessing.Process(target=speak_and_save, args=(text, self.save_path))
        p.start()





def main(args=None):
    multiprocessing.set_start_method('spawn') # Safer for Linux

    rclpy.init(args=args) # initialize ROS2
    node = TTSNode() # create instance of our TTSNode
    rclpy.spin(node) # keep node alive, listening for messages
    node.destroy_node() # cleanup when node shuts down
    rclpy.shutdown() # shut down ROS2 cleanly



# If script is run directly (not imported), run main()
if __name__ == '__main__':
    main()
