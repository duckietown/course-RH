# Duckiebot Setup {#rh-db-setup status=ready}

Excerpt: Learn how to set up your Duckiebot.

Major efforts were made to make sure that the setup of your Duckiebot is as comfortable as possible for you. We
created a set of instructions for initialization and calibration through which we will guide you here.

<div class='requirements' markdown='1'>

Requires: an [assembled Duckiebot](#rh-assembly).

Results: A Duckiebot that is ready to operate in Duckietown.

</div>

<minitoc/>

##Initialization {#rh-db-initialization status=ready}

First of all, you have to flash your SD card. Here you have the possibility to give your duckiebot a name and choose what network to connect to. We experienced people having issues when they called their Duckiebot `duckiebot`, so make sure to find a creative name that is different from that.

Follow the initialization instructions [here](+opmanual_duckiebot#setup-duckiebot).


## Make your Duckiebot move {#rh-make-db-move status=ready}

As soon as you finished the initialization part successfully, it is time to make your Duckiebot move. Follow the instructions [here](+opmanual_duckiebot#rc-control) to find out how you can maneuver your Duckiebot using your computer keyboard. This is also the moment to check whether you did a good job at wiring your motors. If your Duckiebot does not behave as you tell him to, this is probably due to the fact that some wires are crossed.

Note: If this is the first time that you try to make your Duckiebot move, give it some time. It might take some time until the joystick pops up on your screen.  


## See what your Duckiebot sees {#rh-db-camera status=ready}

There is another key component missing now: the image stream from the camera. To find its way around in the city, a Duckiebot needs to be aware of what is going on around him and where he is allowed to drive and where not. To see the image stream from your Duckiebot, follow the instructions [here](+opmanual_duckiebot#read-camera-data).


## Calibration {#rh-calibration status=ready}

As with every real-world system, the hardware of the Duckiebot is always a little different. The "same" cameras or motors that you can buy off the shelf will never be exactly the same. Additionally, the camera might have been mounted in a slightly different orientation than it was supposed to. But don't worry, this is what we are going to take care of in this step.

We have two calibration procedures for the Duckiebot: one for the camera and one for the motors.


### Camera calibration {#rh-camera-calibration status=ready}

The camera calibration procedure consists of two parts: the first one is the intrinsic camera calibration. It accounts for the differences between each camera and is therefore only dependent on the camera itself. If you did the intrinsic calibration, make sure to not play around with the lens of the camera anymore as it will invalidate the intrinsic calibration.

The second part is the extrinsic camera calibration. It accounts for the positioning of the camera relative to its environment (i.e. how you mount it on the Duckiebot). So if you mounted the camera at a slight angle with respect to the driving direction this part accounts for it.

Follow the instructions [here](+opmanual_duckiebot#camera-calib) to calibrate the camera of your Duckiebot.

For more detailed background information check out [this link](https://github.com/duckietown/lectures/blob/master/1_ideal/25_computer_vision/cv_calibration.pdf).

#### Calibration {#exercise:calibration}

During the camera calibration, the Duckiebot will run an automatic verification on the camera calibration. Check if the projection of the street on the actual picture fits. If it doesn't you have to redo the extrinsic calibration.

<end/>

### Wheel calibration {#rh-wheels-calibration status=ready}

The Duckiebot uses a [differential drive](https://docs.duckietown.org/DT19/learning_materials/out/duckiebot_modeling.html). Going forward in a straight line therefore depends on the motors turning at the exact same speed. As in reality every motor is slightly different, we have to account for these imprecisions using a wheel calibration procedure. In Duckietown we are currently using a [gain-trim approach](https://docs.duckietown.org/DT19/learning_materials/out/odometry_calibration.html) for that.

Follow the instructions [here](+opmanual_duckiebot#wheel-calibration) to run through the calibration procedure with your Duckiebot and help him drive straight.
