# Odometry with Wheel Encoders {#odometry-modeling status=ready}

Excerpt: Learn how to work with the wheel encoders from the Duckiebot

<div class='requirements' markdown='1'>
  Requires: [Understanding the DT insfrastructure](#dt-infrastructure)

  Requires: [Working with ROS logs](#ros-logs)

  Results: Being able to work with the wheel encoder data from the Duckiebots
</div>


## Wheel Encoders

Encoders are sensors that are able to convert analog angular position or motion of a shaft into a digital signal. In Duckietown we use [Hall Effect Encoders](https://en.wikipedia.org/wiki/Hall-effect_sensor), which are able to extract the incremental change in angular position of the wheels. This is very useful, since it can be used to accurately map the position of the Duckiebot while it moves in the Duckietown.

Remark: our encoders produce 135 ticks per revolution.

### Encoders in Duckietown

The first task is to get familiar with how encoders work within the Duckietown pipeline. For this you will need a good understanding on how to build your own ROS-compliant Duckietown code. You will be required to create your own subscriber/publisher nodes, get the encoder information, and use it for the following tasks.

Similarly as with the [Braitenberg Vehicles](#exercise:braitenberg-avoiding), we will be developing new Duckietown functionality. For this we will need the following:

- [Creating a basic Duckietown ROS Publisher](#ros-pub-duckiebot)
- [Creating a basic Duckietown ROS Subscriber](#ros-sub-duckiebot)
- [Launch Files](#ros-launch)
- [Namespaces and remapping](#ros-namespace-remap)


Here's a useful (but definitely incomplete!) template.

__Template:__

```python
#!/usr/bin/env python3
import os
import numpy as np
import rospy
import yaml

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped

from std_msgs.msg import Header

class EncoderNode(DTROS):

	def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

		# initialize the DTROS parent class
		super(EncoderNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Set parameters using a robot-specific yaml file if such exists
        self.readParamFromFile()

        # Get static parameters
        self._baseline = rospy.get_param('~baseline')
        self._radius = rospy.get_param('~radius')
        self._k = rospy.get_param('~k')


        # # # #
		# TODO: Initialize variables
        # Note: Think about which variables you need to use for this exercise, and how to initialize them for using them with your code
	    # # # #

        # Subscribing to the wheel encoders
		self.sub_encoder_ticks =rospy.Subscriber("left_wheel_encoder_node/tick", WheelEncoderStamped, self.left_encoder_data, queue_size=1)
		self.sub_encoder_ticks =rospy.Subscriber("right_wheel_encoder_node/tick", WheelEncoderStamped, self.right_encoder_data, queue_size=1)

    # self.sub_encoder_ticks =rospy.Subscriber("wheels_driver_node/wheels_cmd_executed", WheelsCmdStamped, self.YOUR_FUNCTION_HERE, queue_size=1) # Might be useful.

    self.log("Initialized")

  def left_encoder_data(self, ...):

    return


  def right_encoder_data(self, ...):

    return

	def save_encoder_info(self, ...):
		# Might be useful to create a rosbag here
		return



if __name__ == '__main__':
	node = EncoderNode(node_name='my_wheel_encoder_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
	rospy.loginfo("wheel_encoder_node is up and running...")

```
<end/>

#### Get Wheel Encoder Data {#exercise:wheel-encoder-ros-bag}
Do the following:

- Create a copy of the Duckietown ROS template

- Create a subscriber node that is able to obtain the encoder information from both encoders.

- Run your node and the Keyboard Control node.

- Manually drive your Duckiebot around for ~10 seconds, and record a rosbag with the following parameters: encoder ticks (left and right), wheel commands.

Note: for debugging it might be useful to log any relevant information to the screen.  

<end/>

#### Converting Wheel Encoder Information into Distance {#exercise:wheel-encoder-tick-to-meter}

Do the following:

- Modify your previous code to also output the distance travelled by the Duckiebot.

- Publish the distance travelled per wheel to a new topic.

- Manually drive your Duckiebot for ~10 seconds, and record a rosbag with the following values: wheel commands (left, right), encoder ticks (left, right), distance traveled per wheel (left, right).

<end/>
