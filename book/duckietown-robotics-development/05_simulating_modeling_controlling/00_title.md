# [RH5] Simulating and Modeling the Duckiebot {#part:simulating-modeling-controlling status=ready}

Excerpt: Learn how to get started with the Duckietown simulator, create a ROS wrapper for it and practice representation and modeling techniques for the dynamics of the Duckiebot,

At this point you are familiar with implementing basic functionality for autonomous driving and know how to deploy this functionality to the Duckiebot. While this is a great foundation for implementing more complex behavior, it will not suffice for driving autonomously (and safely!) around Duckietown. 

To be able to accomplish this, we will need more fine-tuned and robust control than what was implemented in the [Braitenberg vehicle](https://en.wikipedia.org/wiki/Braitenberg_vehicle) controller. As we will see, this will require us to be able to represent the state of the robot in some way, to model the dynamics of this state and to identify the parameters of this model. We will also need a way of testing complex behavior in a safe manner without having to risk the life of Duckies or without needing a physical Duckietown or Duckiebot. This is where a simulator will really come in handy.

<minitoc/>
