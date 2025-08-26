# this is the launch file
# it will be used to execute the tts node operation
# you can change the message that the publisher sends out as well



from launch import LaunchDescription # imports the main LaunchDescription class which defines what will be launched when we run the file
from launch_ros.actions import Node # Node is how we will define a ROS 2 node that will be launched


# this is the required function in every launch file:
# ROS 2 will call this function to know the nodes and settings to launch

def generate_launch_description():

	# this will return a LaunchDescription object containing a list of launch actions which define the nodes to start and how they will start
	return LaunchDescription([
		
		# this starts the publisher node:
		Node(
			package = 'tts_package', # which ROS 2 package the node belong to
			executable = 'tts_publisher', # name of the script registered in setup.py under console_scripts
			name = 'tts_publisher', # the name the node will have in the ROS 2 system
			parameters = [{'message': 'This is a top secret message!'}] # overrides the default message parameter aka allows you to change the message published
		),
		
		# this starts the subscriber node:
		Node(
			package = 'tts_package',
			executable = 'tts_node',
			name = 'tts_node'
			# este node não precisa de parameters, porque só vai receber do topic
		),
	])