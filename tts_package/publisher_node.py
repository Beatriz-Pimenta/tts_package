# this is the publisher node
# it will create a publisher to the /tts_input topic and send out a message every 3 seconds (with the help of a timer)
# it will also log everything that is published


import rclpy # imports the ROS 2 Python client library
from rclpy.node import Node # imports the base class used to create ROS 2 nodes
from std_msgs.msg import String # imports the String message type, which is used to send simple text messages


# defines a class TextPublisher that inherits from Node, making it a ROS 2 node:

class TextPublisher(Node):
	def __init__ (self):
		super().__init__('publisher_node') # inicializes the base class with the noe name 'publisher_node'
		
		# declares a ROS 2 parameter named message, which will allow us to change the message from the command line:
		# (its default value will be 'Estás a ouvir?')
		self.declare_parameter('message', 'Are you listening?') 
		
		# this will create a publisher:
		self.publisher_ = self.create_publisher(
			String, # message type
			'/tts_input', # topic to which the messages will be published
			10 # queue size for messages waiting to be sent
		)
		
		# this will set up the timer:
		timer_period = 3.0 # seconds until next message is sent
		self.timer = self.create_timer(timer_period, self.timer_callback) # call the timer_callback function every [timer_period] seconds
	
	
	def timer_callback(self):
		msg = String() # creates a new message of type String
		
		# this will read the current value of the message parameter and store it in the data field of the message
		msg.data = self.get_parameter('message').get_parameter_value().string_value
		
		self.publisher_.publish(msg) # publishes the message to the /tts_input topic
		self.get_logger().info(f'Publishing: "{msg.data}"') # logs the message's content to the terminal for visibilty
		
		
		
def main(args=None):
	rclpy.init(args=args) # initializes the ROS 2 client library
	node = TextPublisher() # creates an instance of the TextPublisher node
	
	try:
		rclpy.spin(node) # starts processing callbacks and keeps the node alive
	except KeyboardInterrupt:
		node.get_logger().info('Keyboard interrupt received. Shutting down...')
	finally:
		node.destroy_node() # destroys the node
		rclpy.shutdown() # shuts down
	
if __name__ == '__main__':
	main()
	
