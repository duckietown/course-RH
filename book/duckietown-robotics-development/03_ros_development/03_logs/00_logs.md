

# Working with logs {#ros-logs status=ready}

Robotics is innately married to hardware. However, when we develop and test our robots' software, it is often the case that we don't want to have to waste time to test on hardware after every small change. With bigger and more powerful robots, it might be the case that a software can result in a robot actuation that breaks it or even endanger human life! But if one can evaluate how a robot or a piece of code would behave before deploying on the actual platform then quite some headaches can be prevented. That is why working in simulation and from logs is so important in robotics. In this section you will learn how to work with logs in ROS.


<div class='requirements' markdown='1'>
  Requires: [Laptop setup](+opmanual_duckiebot#laptop-setup)
  
  Requires: [Docker poweruser skills](#docker-poweruser)
  
  Requires: [Developer knowledge of ROS](#dt-infrastructure) 
  
  Results: Reading and processing bag files
</div>

<minitoc/>


## Rosbag {#rosbag}


A bag is a file format in ROS for storing ROS message data. Bags, named so because of their `.bag` extension, have an important role in ROS. Bags are typically created by a tool like `rosbag`, which subscribes to one or more ROS topics, and stores the sequence of messages in a file as it is received. These bag files can be played back in ROS ith the same topics that were recorded, or even using remapping to new topics. When a bag file is replayed the temporal order of the different messages is always kept.


Please go through [this](http://wiki.ros.org/rosbag/Commandline) link for more information.

## Rosbag: Recording {#rosbag-record}

You can use the following command to record bag files

    $ rosbag record TOPIC_1 TOPIC_2 TOPIC_3

or simply

    $ rosbag record -a

to record all messages being published. 

Note: Be careful on recording all the messages published in a ROS system. There might be quite a lot of topics creating very by bag files very quickly, especially using images.


## Rosbag Python API: Reading {#rosbag-read}

The following code snippet is a basic usage of the `rosbag` API to read bag files:

```python
import rosbag
bag = rosbag.Bag('test.bag')
for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
    print msg
bag.close()
```

## Rosbag Python API: Writing {#rosbag-write}

The following code snippet is a basic usage of the `rosbag` API to create bag files:

```python
import rosbag
from std_msgs.msg import Int32, String

bag = rosbag.Bag('test.bag', 'w')

try:
    s = String()
    s.data = 'foo'

    i = Int32()
    i.data = 42

    bag.write('chatter', s)
    bag.write('numbers', i)
finally:
    bag.close()
```

## Exercises {#rosbag-exercise}

All containers in the exercises below should be run on your laptop, i.e. without `-H ![MY_ROBOT].local`.


#### Record bag file {#exercise:rosbag-record-bag}

Using the following concepts, 

- [Getting data in and out of your container](#docker-poweruser)
- [Communication between laptop and Duckiebot](#ros-multi-agent)

create a Docker container on your laptop with a folder mounted on the container. You can use the image `duckietown/dt-ros-commons:daffy`. This time, however, instead of exporting the `ROS_MASTER_URI` and `ROS_IP` after entering the container, do it directly with the `docker run` command specifying environment variables. You already know how from [here](#exercise:ex-docker-envvar).

Run the [lane following demo](+opmanual_duckiebot#demo-lane-following). Once your Duckiebot starts moving, record the camera images and the wheel commands from your Duckiebot using `rosbag` in the container you just created (the one with the folder mounted). In order to save the bags once the container is stopped you should record them in the mounted folder. To do that navigate to the mounted folder using the `cd` command and then run

    laptop-container $ rosbag record /![MY_ROBOT]/camera_node/image/compressed /![MY_ROBOT]/wheels_driver_node/wheels_cmd
  
Record the bag file for 30 seconds and then stop the recording using <kbd>Ctrl</kbd>+<kbd>C</kbd>. Use the `rosbag info ![filename].bag` command to get some information about the bag file. If the bag does not have messages from both the topics, check if you ran the container correctly(you can easily check that a topic is published using the `rostopic echo` functionality from within the container).

Stop the demo before proceeding.

<end/>

#### Analyze bag files {#exercise:rosbag-stats}

Download [this](https://www.dropbox.com/s/11t9p8efzjy1az9/example_rosbag_H3.bag?dl=1) bag file. 

Start by creating a new repository from the template, like in [section C-2](#basic-structure). Inside, the `./packages` folder, create a python file for this exercise. You do not need to create a ros package for this, you can simply launch a python script as you did in RH2. This is because reading a bag file does not actually require ROS, however, you can still choose to do so if you want. Using the following concepts,

- [Getting data in and out of your container](#docker-poweruser)
- [Creating a basic Duckietown ROS enabled Docker image](#basic-structure)


create a Docker image which can analyze bag files and produce an output similar to the one shown below. The min, max, average, and median values printed are statistics of the time difference between two consecutive messages. The `NNN` and `N.NN` are just placeholders, eg. `NNN` could be 100 and `N.NN` could be 0.05. 

```
/tesla/camera_node/camera_info:
  num_messages: NNN
  period:
    min: N.NN
    max: N.NN
    average: N.NN
    median: N.NN

/tesla/line_detector_node/segment_list:
  num_messages: NNN
  period:
    min: N.NN
    max: N.NN
    average: N.NN
    median: N.NN
    
/tesla/wheels_driver_node/wheels_cmd:
  num_messages: NNN
  period:
    min: N.NN
    max: N.NN
    average: N.NN
    median: N.NN

``` 


Note: Make sure to mount the folder containing the bag file to the Docker container, instead of copying it. 

Run the same analysis with the bag file you recorded in the previous exercise.

<end/>

#### Processing bag files {#exercise:rosbag-process}

Use the bag file which you recorded earlier for this exercise. Using the following concepts, 

- [Getting data in and out of your container](#docker-poweruser)
- [Creating a basic Duckietown ROS enabled Docker image](#basic-structure)
- [Converting between ROS Images and OpenCV Images](http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython)

create a Docker image which can process a bag file. Essentially, you will extract some data from a bag file, process it, and write the results to a new bag file. Once again, create a new repository, and the necessary python file for this exercise inside the `./packages` folder. For the image message in the bag file, do the following:

- Extract the timestamp from the message
- Extract the image data from the message
- [Draw](https://docs.opencv.org/2.4/modules/core/doc/drawing_functions.html#puttext) the timestamp on top of the image
- Write the new image to the new bag file, with the same topic name, same timestamp, and the same [message type](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CompressedImage.html) as the original message

The new bag file should be generated in the mounted folder.

To verify your results, create a docker container exactly like you did in [exercise 18](#exercise:rosbag-record-bag). Make sure you place your processed bag file in the folder being mounted. Run the following command:

    laptop-container $ rosbag play ![processed_bag].bag --loop /![MY_ROBOT]/camera_node/image/compressed:=/new_image/compressed

In a new terminal, use `dts start_gui_tools` to open a container connected to your robot and run `rqt_image_view` inside it. Can you see `/new_image/compressed`? 

Stop the `rosbag play` using <kbd>CTRL</kbd>+<kbd>C</kbd> and now run the following command inside the same container:

    laptop-container $ rosbag play ![processed_bag].bag --loop 

Again, use `start_gui_tools` but this time check `/![MY_ROBOT]/camera_node/image/compressed`. What's going on? Why? What does the last part of the first command do?

<end/>
