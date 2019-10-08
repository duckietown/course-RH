# [RH4] Implementing basic robot behaviors {#part:basic-robot-behavior status=ready}

Excerpt: Learn how to crete new robot behavior by reusing parts of the Duckitown code 

You are already master of Docker and ROS and you can make small ROS programs that run on your robot! This is pretty nice but does it mean you need to write everything from scratch if you want to change or improve an existing demo or functionality? Not the least bit!

Adding functionality to your Duckiebot while reusing the ROS nodes that are already implemented is incredibly easy and intuitive. That is where ROS and Docker really come in handy. In this module, we will do exactly that. We will use the already existing ROS nodes that control the camera, wheels, and LEDs of your robot and will implement a [Braitenberg vehicle](https://en.wikipedia.org/wiki/Braitenberg_vehicle) controller on top of them. But first, we will take a look at how Duckietown's code is organized.

<minitoc/>
