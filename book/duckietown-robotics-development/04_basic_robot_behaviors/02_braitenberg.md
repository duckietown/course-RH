# Developing new Duckiebot functionality {#new-duckiebot-functionality status=ready}


Excerpt: Learn how to develop new functionality within the Duckietown framework 

You will now learn how to add your own code to already existing Duckietown codebase. In particular you will learn how to interface your nodes with the provided ones such that you don't have to rewrite already existing modules. Then, you will be able to master these skills by developing [Braitenberg vehicle](https://en.wikipedia.org/wiki/Braitenberg_vehicle) behavior on Duckiebots.

<div class='requirements' markdown='1'>
  Requires: [Docker basics](#docker-basics)

  Requires: [ROS basics](#part:sw-advanced)
  
  Requires: [Knowledge of the software architecture on a Duckiebot](#duckietown-code-structure)

  Results: Skills on how to develop new code as part of the Duckietown framework
</div>

<minitoc/>

## Exploring DTROS

The `DTROS` class is often referred to as the 'mother node' in Duckietown. It provides some very useful functionalities that the other nodes inherit. It has modified ROS Subscribers and Publishers which can be switched on and off. It also provides an interface to the ROS parameters of the node using it which allows dynamical changes while the node is running. For this reason we strongly suggest you to always base your nodes on `DTROS`. Instead of explaining all the details of `DTROS`, we instead invite you to investigate them yourself.

Note: Currently `dt-core` is not using `DTROS`. Nevertheless, soon the nodes there will be converted to the `DTROS` framework as well.

#### Exploring how DTROS works {#exercise:exploring-dtros}

First, take a look at the documentation of `DTROS` [here](http://rosapi.duckietown.p-petrov.com/repositories/dt-ros-commons/docs/source/packages/duckietown.html#duckietown.DTROS). Find out how its functionalities are implemented by looking at their implementation in the `dt-ros-commons` repository [here](https://github.com/duckietown/dt-ros-commons/tree/daffy/packages/duckietown/include/duckietown). In particular, make sure you can answer the following list of questions. To do that, it might be helpful to see how `DTROS` is being used in some of the other nodes. Take a look at [`camera_node`](https://github.com/duckietown/dt-duckiebot-interface/blob/daffy/packages/camera_driver/src/camera_node.py), the [`wheels_driver_node`](https://github.com/duckietown/dt-duckiebot-interface/blob/daffy/packages/wheels_driver/src/wheels_driver_node.py), and the other nodes in `dt-duckiebot-interface` and `dt-car-interface`.

- How do you initialize the `DTROS` parent class? How do you start your node? What does `rospy.spin()` do? (_Hint: look at the nodes in `dt_duckiebot_interface`_)

- When should you redefine the `onShutdown` method? Why do you still need to call the `onShutdown` method of `DTROS`? (_Hint: look at the nodes in `dt_duckiebot_interface` and at the official ROS documentation_)

- What is the difference between the `DTROS` `log` method  and the native ROS logging?

- How are the parameters dynamically updated? What can you do to the frequency at which this happens? Why is `updateParameters` called immediately after initializing the parameters? What is the use of the `parametersChanged` attribute? (_Hint: see the implementation in [`camera_node`](https://github.com/duckietown/dt-duckiebot-interface/blob/daffy/packages/camera_driver/src/camera_node.py)_) 

- Should you ever use `rospy.get_param()` in your node? If not, how should you access a ROS parameter? How do you initialize the parameters of your node? (_Hint: look at the nodes in `dt_duckiebot_interface` and at the official ROS documentation_)

- What does the `~switch` service do? How can you use it? What is the benefit of using it?

- What is the difference between the native ROS Subscriber and Publisher and `DTPublisher` and `DTSubscriber`?

<end/>

## Basic Braitenberg vehicle behavior

Through a series of exercises you will implement a very basic brightness- and color- based controller for your Duckiebot that can result in a surprisingly advanced robot behavior. In his book _Vehicles: Experiments in Synthetic Psychology_, Valentino Braitenberg describes some extremely basic vehicle designs that are capable of demonstrating complex behaviors. By using only a pair of 'sensors' that can only detect brightness, two motors, and direct links between the sensors and the motors, these vehicles can exhibit love, aggression, fear, foresight and many other complex traits.

<figure> 
  <figcaption>Avoiding and attracting Braitenberg behavior (illustration from [Thomas Schoch](https://commons.wikimedia.org/wiki/File:Braitenberg_Vehicle_2ab.png))</figcaption>
  <img style="width:15em" src="images/Braitenberg_Vehicle.png" />
</figure>

In the image above, the light intensity detected by a sensor is used proportionally to control a motor. Depending on whether  each sensor is connected to the motor on the same or the opposite side, respectively avoiding or attracting behavior can be observed. These behaviors can further be combined if the robot also detects the color of the light.

Here's an example video of how this Braitenberg behavior would look like on Duckiebots. When the light a Duckiebot sees is green, it has attracting behavior. Otherwise, it will be avoiding. By the end of this series of exercises you will be able to create similar Duckiebot controllers. Note that while this is recorded in a dark room, with a few smart tricks you can also make your robots work in well-lit spaces.

<figure id="video-braitenberg">
    <dtvideo src="vimeo:365020910"/>
</figure>


#### Avoiding Braitenberg vechicles {#exercise:braitenberg-avoiding}

Using everything you have learnt so far, create a ROS node that implements the avoiding Braitenberg behavior. You should run this ROS node in a container running on your Duckiebot. Here are some details and suggestions you might want to take into account:

- Use the `dt-duckiebot-interface` and all the drivers it provides. In particular, you will need to subscribe to the images that the `camera_node` publishes and to publish wheel commands to `wheel_driver_node`. To do that simply make sure that the `dt-duckiebot-interface` container is running. Then, whenever you start the container with your code and `--net host`(why?), they will share their ROS Master, so that your subscribers and publishers can find each other.

- Use the nodes in `dt-duckiebot-interface` as a reference for code and documentation style. You will find a number of useful code snippets there.

- Use the [ROS template](https://github.com/duckietown/template-ros) and create your package and node there. Don't forget to add the `package.xml` and `CMakeLists.txt` files, and to make your Python code executable, as explained [before](#ros-pub-laptop).

- Your controller needs to run in real time with a frequency of at least 10-12 Hz. Therefore, processing the input image at its full resolution might not be possible and you should consider reducing it. A neat way to do this is to change the configuration parameters of the `camera_node` running in `dt-duckiebot-interface`. In the template node code below that is already done for the exposure mode. Consult the [ROS API docs](http://rosapi.duckietown.p-petrov.com/repositories/dt-duckiebot-interface/docs/source/packages/camera_driver.html#cameranode) for the `CameraNode` class if you are not sure about which parameters you can change.

- For now ignore the color that your bot observes, focus only on the brightness. If you still want to change the color of the LEDs, use the `set_pattern` service provided by the `led_emitter_node`. Its use is also documented on the [ROS API docs](http://rosapi.duckietown.p-petrov.com/repositories/dt-duckiebot-interface/docs/source/packages/led_emitter.html#ledemitternode). You do not need to call this service from inside your python file. You can use the `start_gui_tools` functionality of the duckietown-shell to give you a terminal to run your service call in. 

- You will need to publish `WheelsCmdStamped` messages to `wheel_driver_node`. You can see the message structure [here](https://github.com/duckietown/dt-ros-commons/blob/daffy/packages/duckietown_msgs/msg/WheelsCmdStamped.msg).

- The template loads the kinematics calibration on your Duckiebot so you don't need to worry about trimming your Braitenberg controller. Simply use the provided `speedToCmd` method apply gain, trim, and the motor constant to your wheel commands. 

- If your Duckiebot keeps on moving even after you stop your node, you will have to edit the provided `onShutdown` method. Make sure that the last commands your node publishes to `wheel_driver_node` are zero.

- Once you have finished this exercise, you should have a Duckiebot which goes towards the left if your program senses that the right side has more brightness, and vice versa.

__Template:__

```python
#!/usr/bin/env python

import cv2
import numpy as np
import os
import rospy
import yaml

from duckietown import DTROS
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage, CameraInfo
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped


class BraitenbergNode(DTROS):
    """Handles the imagery.

    This node implements Braitenberg vehicle behavior on a Duckiebot.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Configuration:
        ~gain (:obj:`float`): scaling factor applied to the desired velocity, taken from the 
            robot-specific kinematics calibration 
        ~trim (:obj:`float`): trimming factor that is typically used to offset differences in the
            behaviour of the left and right motors, it is recommended to use a value that results in
            the robot moving in a straight line when forward command is given, taken from the 
            robot-specific kinematics calibration 
        ~baseline (:obj:`float`): the distance between the two wheels of the robot, taken from the 
            robot-specific kinematics calibration 
        ~radius (:obj:`float`): radius of the wheel, taken from the robot-specific kinematics calibration 
        ~k (:obj:`float`): motor constant, assumed equal for both motors, taken from the 
            robot-specific kinematics calibration 
        ~limit (:obj:`float`): limits the final commands sent to the motors, taken from the 
            robot-specific kinematics calibration 

    Subscriber:
        ~image/compressed (:obj:`CompressedImage`): The acquired camera images

    Publisher:
        ~wheels_cmd (:obj:`duckietown_msgs.msg.WheelsCmdStamped`): The wheel commands that the motors will execute

    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(BraitenbergNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_namespace().strip("/")

        # Use the kinematics calibration for the gain and trim
        self.parameters['~gain'] = None
        self.parameters['~trim'] = None
        self.parameters['~baseline'] = None
        self.parameters['~radius'] = None
        self.parameters['~k'] = None
        self.parameters['~limit'] = None

        # Set parameters using a robot-specific yaml file if such exists
        self.readParamFromFile()
        self.updateParameters()

        # Wait for the automatic gain control
        # of the camera to settle, before we stop it
        rospy.sleep(2.0)
        rospy.set_param('/%s/camera_node/exposure_mode'%self.veh_name, 'off')

        self.log("Initialized")

    def speedToCmd(self, speed_l, speed_r):
        """Applies the robot-specific gain and trim to the output velocities

        Applies the motor constant k to convert the deisred wheel speeds to wheel commands. Additionally,
        applies the gain and trim from the robot-specific kinematics configuration.

        Args:
            speed_l (:obj:`float`): Desired speed for the left wheel (e.g between 0 and 1)
            speed_r (:obj:`float`): Desired speed for the right wheel (e.g between 0 and 1)

        Returns:
            The respective left and right wheel commands that need to be packed in a `WheelsCmdStamped` message

        """

        # assuming same motor constants k for both motors
        k_r = self.parameters['~k']
        k_l = self.parameters['~k']

        # adjusting k by gain and trim
        k_r_inv = (self.parameters['~gain'] + self.parameters['~trim']) / k_r
        k_l_inv = (self.parameters['~gain'] - self.parameters['~trim']) / k_l

        # conversion from motor rotation rate to duty cycle
        u_r = speed_r * k_r_inv
        u_l = speed_l * k_l_inv

        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = self.trim(u_r, -self.parameters['~limit'], self.parameters['~limit'])
        u_l_limited = self.trim(u_l, -self.parameters['~limit'], self.parameters['~limit'])

        return u_l_limited, u_r_limited

    def readParamFromFile(self):
        """
        Reads the saved parameters from `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml` or
        uses the default values if the file doesn't exist. Adjsuts the ROS paramaters for the node
        with the new values.

        """
        # Check file existence
        fname = self.getFilePath(self.veh_name)
        # Use the default values from the config folder if a robot-specific file does not exist.
        if not os.path.isfile(fname):
            self.log("Kinematics calibration file %s does not exist! Using the default file." % fname, type='warn')
            fname = self.getFilePath('default')

        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s" %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return

        # Set parameters using value in yaml file
        if yaml_dict is None:
            # Empty yaml file
            return
        for param_name in ["gain", "trim", "baseline", "k", "radius", "limit"]:
            param_value = yaml_dict.get(param_name)
            if param_name is not None:
                rospy.set_param("~"+param_name, param_value)
            else:
                # Skip if not defined, use default value instead.
                pass

    def getFilePath(self, name):
        """
        Returns the path to the robot-specific configuration file,
        i.e. `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml`.

        Args:
            name (:obj:`str`): the Duckiebot name

        Returns:
            :obj:`str`: the full path to the robot-specific calibration file

        """
        cali_file_folder = '/data/config/calibrations/kinematics/'
        cali_file = cali_file_folder + name + ".yaml"
        return cali_file

    def trim(self, value, low, high):
        """
        Trims a value to be between some bounds.

        Args:
            value: the value to be trimmed
            low: the minimum bound
            high: the maximum bound

        Returns:
            the trimmed value
        """

        return max(min(value, high), low)

    def onShutdown(self):
        """Shutdown procedure.

        Publishes a zero velocity command at shutdown."""

        # MAKE SURE THAT THE LAST WHEEL COMMAND YOU PUBLISH IS ZERO, OTHERWISE YOUR
        # DUCKIEBOT WILL CONTINUE MOVING AFTER THE NODE IS STOPPED

        # PUT YOUR CODE HERE

        super(BraitenbergNode, self).onShutdown()


if __name__ == '__main__':
    # Initialize the node
    camera_node = BraitenbergNode(node_name='braitenberg')
    # Keep it spinning to keep the node alive
    rospy.spin()
```
<end/>

#### Attracting Braitenberg vechicles {#exercise:braitenberg-attracting}

You should be able to change the avoiding behavior of your robot into an attracting one by editing just a few lines of code. Give it a try! Once you have finished this exercise, you should have a Duckiebot which goes towards the right if your program senses that the right side has more brightness, and vice versa.

<end/>

#### Combined behavior Braitenberg vechicles {#exercise:braitenberg-combined}

Add a color detector to your Braitenberg controller node. If your Duckiebot sees green light (perhaps of a different Duckiebot) it should be attracted to it, otherwise it should be repelled by it.

If you have more than one robot, try to run your controller on a few of them. Set some to have green LEDs, and some red. Do you see complex behavior emerging? Changing the color of the LEDs can be done with the `set_pattern` service provided by the `led_emitter_node` in `dt-duckiebot-interface`. It is documented on the [ROS API docs](http://rosapi.duckietown.p-petrov.com/repositories/dt-duckiebot-interface/docs/source/packages/led_emitter.html#ledemitternode).

Can you devise even more complex behavior and interactions? 
