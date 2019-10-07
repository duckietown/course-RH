# Robot behaviour with ROS {#ros-robot-behaviour status=ready}

In this section you will extend an [earlier exercise](#exercise:ex-docker-colordetector) to work with ROS.


<div class='requirements' markdown='1'>
  
  Requires: [Laptop setup](+opmanual_duckiebot#laptop-setup)
  
  Requires: [Duckiebot initialization](+opmanual_duckiebot#setup-duckiebot)
  
  Requires: [Docker poweruser skills](#docker-poweruser)

  Requires: [Developer knowledge of ROS](#dt-infrastructure) 
  
  Results: Basic robot behavior with ROS

</div>

## ROS based color detector

#### Converting the color detector to ROS nodes {#exercise:ros-color-detector}

Using the following concepts:

- [Creating a basic Duckietown ROS Publisher](#ros-pub-duckiebot)
- [Creating a basic Duckietown ROS Subscriber](#ros-sub-duckiebot)
- [Launch Files](#ros-launch)
- [Namespaces and remapping](#ros-namespace-remap)
- [Multi agent communication](#ros-multi-agent)
- [Recording bag files](#rosbag-record)

do the following:

- Create two repositories from the ROS template. 

- Add all your python dependencies to the file `./dependencies-py.txt`

- In the first one, add the code to extract images using a PiCamera and publish it on a topic. This will run on your Duckiebot. The node should run using a launch file. Remember to turn off `duckiebot-interface` (can exist under different names) and any other container which can use the camera.

- In the second one, add the code to subscribe to that topic and extract color. Using concepts from [roslaunch](#ros-launch), create two nodes in your `.launch` file. Note that you are not allowed to have different Python files for each node. The first node detects the color red and the second detects yellow. You should use [params](http://wiki.ros.org/roslaunch/XML/param) within your `node` tag to let your detector know whether it is supposed to detect red/yellow. These nodes will run on your laptop. Once again, pass the required environment variables to connect your laptop to the rosmaster of your duckiebot using `docker run`.

- You should publish some debug images from within the color detection node. These debug images should have rectangles drawn in the region where the colors are detected. Note that we are not looking for perfect color detectors, as long as they produce reasonable output. You can draw multiple rectangles in the image if the multiple regions in the image have the requred color.

- If you are using `sensor_msgs/CompressedImage`, make sure that your image topic names end with `/compressed`. For example, instead of naming the topic `/my_image`, name it `/my_image/compressed`

- Record a bag file containing the original and debug images. 

A sample debug image stream for the yellow color detector is shown here:
<figure id="example-embed">
    <figcaption>Sample Yellow Color Detector</figcaption>
    <dtvideo src="vimeo:364266236"/>
</figure>

<end/>
