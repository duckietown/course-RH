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

The data from each wheel encoders can be used to determined the distance travelled by each wheel:

$$ \Delta X = 2 \pi R {N_{ticks}/N_{total}} $$

  - $$ \Delta X $$ is the distance travelled by each wheel;
  - $$ N_{ticks} $$ is the number of ticks measured from each wheel;
  - $$ N_{total} $$ is the number of ticks in one full revolution (in our case that's 135).

Below you can see the `WheelEncoderStamped.msg` from the Duckietown messages package:

```
# Enum: encoder type
uint8 ENCODER_TYPE_ABSOLUTE = 0
uint8 ENCODER_TYPE_INCREMENTAL = 1

Header header
uint32 data
uint16 resolution
uint8 type
```

Now you are ready to get started with the Duckietown encoders!
Here's a useful (but definitely incomplete!) template that will help you get going. Note, this is not the only solution. Any solution which implements the required functionality will be considered correct. 

__Template:__

```python
#!/usr/bin/env python3
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32

class OdometryNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(EncoderNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)

        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber(...)
        self.sub_encoder_ticks_right = rospy.Subscriber(...)
        self.sub_executed_commands = rospy.Subscriber(...)

        # Publishers
        self.pub_integrated_distance_left = rospy.Publisher(...)
        self.pub_integrated_distance_right = rospy.Publisher(...)

        self.log("Initialized")

    def cb_encoder_data(self, wheel, msg):
        """ Update encoder distance information from ticks.
        """

    def cb_executed_commands(self, msg):
        """ Use the executed commands to determine the direction of travel of each wheel.
        """
        
if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")

```
<end/>

#### Get Wheel Encoder Data {#exercise:wheel-encoder-ros-bag}

Do the following:

- Create a copy of the Duckietown ROS template.

- Create a subscriber node that is able to obtain the encoder information from both encoders.

- Run your node and the Keyboard Control node.

- Manually drive your Duckiebot around for ~10 seconds, and record a rosbag with the following parameters: encoder ticks (left and right), wheel commands.

Note: You could record the data from the topics directly with a rosbag (if Keyboard Control is running), but creating the subscriber node is necessary for the next step.  

<end/>

#### Converting Wheel Encoder Information into Distance {#exercise:wheel-encoder-tick-to-meter}

Do the following:

- Modify your previous code to also output the distance travelled by each wheel of the Duckiebot. Tip: this can be done by integrating the distance traveled by each wheel, but you need to take care of the direction of rotation of the wheels.

- Publish the distance travelled per wheel to a new topic.

- Manually drive your Duckiebot for ~10 seconds, and record a rosbag with the following values: wheel commands (left, right), encoder ticks (left, right), distance traveled per wheel (left, right).

<end/>
