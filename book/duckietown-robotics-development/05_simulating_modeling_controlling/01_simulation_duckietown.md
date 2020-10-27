# Simulation in Duckietown {#duckietown-simulation status=ready}

Excerpt: Setup, connect and operate the Duckietown simulator.

<div class='requirements' markdown='1'>
  Requires: [Implementing Basic Robot Behaviors](#part:basic-robot-behavior)
  
  Results: Experience with running and testing on the Duckietown simulator 
</div>

<minitoc/>

## Why simulation?
Daphne is an avid Duckietowner who loves Duckies. In her mission to "save the Duckies" from bugs in her code she used to spend a large portion of her time writing unit tests for her algorithms and ROS nodes. Some of these tests would check that the accuracy of her object detection pipeline was above a certain threshold, that the estimated offset of the Duckiebot from the lane given several input images was correct or that the output of the controller given several offsets gave sensible results. She noticed that this way of testing would fall short in several aspects:

- The number of hand-crafted edge cases was not representative of the number of situations the Duckiebot would encounter in a single drive
- Issues at the interface of these algorithms would not be caught
- To increase code coverage and maintain it, a lot of time would need to go into writing tests, mock ups, gathering and labelling test data, etc
- Quantifying controller performance was hard without having access to a model of the vehicle used to propagate the state forward in time

Daphne also found that having to charge her robot's battery, setting up her Duckietown loop, placing her Duckiebot on the loop, connecting to it, and running the part of the pipeline that had to be tested everytime she or someone in her team wanted to merge new changes into the codebase was extremely time consuming.

More over, Daphne and her real Duckiebot only have access to a small Duckietown loop. But she wants to ensure that her algorithms work in the most complicated and busy environments of Duckietown.

All of the above were compelling reasons for Daphne to start looking at full-stack simulators that would allow her to simultaneously address the shortcomings of unit testing, the inconvenience of manual testing and the ability to test scenarios that are not possible or too risky in real life. 

Luckily, she found just the right thing at the [Duckietown gym](https://github.com/duckietown/gym-duckietown).

Daphne's story is the story of every autonomous driving company, whose mission is instead to "save the humans" and which cannot afford to make mistakes on the real roads, and which require automated integration testing tools that can be run faster-than-real-time under challenging conditions. As an example, Waymo has driven around 20 million miles on real roads, but around 15 billion miles in simulation!

## The Duckietown Simulator

In this part of the exercise, you will become familiar with the Duckietown simulator by reading the setup instructions here: [](+AIDO#dt-simulator) and driving a robot around a virtual city. Of course, you are welcome to try the other many features of this simulator.

To demystify the simulator, here are a few tips to get started.

To run a minimal demo of the simulator, you simply need a (virtual) environment with the gym_duckietown pip3 package installed.

To setup such an environment, the safest way is to run the following (you could also skip the virtual environment but you may have clashing packages installed):

    laptop $ cd ~ && virtualenv dt-sim
    
    laptop $ source dt-sim/bin/activate

    laptop $ pip3 install duckietown-gym-daffy

Now you need to create a simple python script with uses the gym-duckietown api to connect to the simulator, the API is very simple as you will see.

Create and run the following file, from within the environment you have setup above:

```python
#!/usr/bin/env python3
import gym_duckietown
from gym_duckietown.simulator import Simulator
env = Simulator(
        seed=123, # random seed
        map_name="loop_empty",
        max_steps=500001, # we don't want the gym to reset itself
        domain_rand=0,
        camera_width=640,
        camera_height=480,
        accept_start_angle_deg=4, # start close to straight
        full_transparency=True,
        distortion=True,
    )   
while True:
    action = [0.1,0.1]
    observation, reward, done, misc = env.step(action)
    env.render()
    if done:
        env.reset()
```

What do you observe? Does this make sense? Why is it driving straight? Can you make it drive backwards or turn? When is `done = True`? What is `observation`? 

If you want to drive the robot around in simulation you might have read about the utility script `manual_control.py`. This is located in the root of the [gym_duckietown](https://github.com/duckietown/gym-duckietown) repository and can be run after making sure that all the dependencies are met. Clone the repository and in the root of it run the following:

    laptop $ pip3 install -r requirements.txt
    
    laptop $ pip3 install -e .

now run:

    laptop $ ./manual_control.py --env-name Duckietown-udem1-v0

You should be able to drive around with the arrow keys. If you are experiencing large delays and low frame rate, please replace the lines

```python

pyglet.clock.schedule_interval(update, 1.0 / 30)

# Enter main event loop
pyglet.app.run()

```

by

```python
import time

...

dt = 0.01
while True:
    update(dt)
    time.sleep(dt)

```



