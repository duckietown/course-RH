# Basic Duckiebot operation {#basic-db-operation status=ready}

Excerpt: Learn how to operate the built-in functionality of the Duckiebot.

Now that you know more about how to assemble a duckiebot, how to use a terminal, how to set up a Duckiebot, how to handle a bit of networking and a bit of Docker, it is high time you learn how to use the basic functionalities of the Duckiebot. In this section, you will learn multiple ways to operate and manage existing functions of the Duckiebot.


<!-- !!! UPDATE THIS !!! -->
<div class='requirements' markdown='1'>
  Requires: [Laptop setup](+opmanual_duckiebot#laptop-setup)

  Results: Know how to use the Dashboard, Portainer and the DT shell for demos.
</div>

<minitoc/>


## Remote connection with a browser and an interface  {status=ready}

One of the easiest way to use and get an overview of your Duckiebot's operations capacities is to use a Duckietown designed web interface, that we call the _Dashboard_. The dashboard will allow you to monitor and operate basic functionalities of the Duckiebot.

#### Using the Dashboard {#exercise:dashboard}

To set up the dashboard, follow this [tutorial](+opmanual_duckiebot#duckiebot-dashboard-setup). Once on the dashboard, explore the interface and try to understand its features.

Through the dashboard you can, e.g., move the Duckiebot. You can find a tutorial on how to do so on [](+opmanual_duckiebot#setup-ros-websocket-image).

You can even see what the Duckiebot is seeing Through the dashboard. You can follow the instructions from [](+opmanual_duckiebot#image-dashboard) to do so.

<end/>

The dashboard is really useful for quick debugging and for moving the Duckiebot. We suggest you use it every time you have doubts about the camera nor working or the motors not being plugged in the right way.


But this interface has its limits, as it hides everything that is actually running on the duckiebot. To better understand the duckiebot, let’s take a look at what is under the hood : we will use portainer.

To manage and use containers, the command line interface is not so easy to use. But there exist a tool that create a nice interface to manage containers: [_Portainer_](https://www.portainer.io/). Portainer is itself a container that runs on a device. Let's learn how to use it.

#### Using Portainer {#exercise:portainer}

Luckily, We have one running directly on the duckiebots at startup. Go to `![hostname].local:9000` on your web browser. You should arrive on an interface. Navigate on the side window to `Containers`. Here you will see all the containers that are running or that are stopped on your duckiebot.

Look for the one that has `duckiebot_interface` in the name. This one contains all the drivers you need to drive around, use the camera and the leds.

Select it, click on stop, then try to move your duckiebot around again with the dashboard. It doesn’t work anymore. Select it again and start it. Now, find the `logs` button, right next to the name. This will open the logs output of the container. This can be very useful to debug new containers. In here you might see the error messages if something goes wrong.

<end/>

With this interface, you can also attach a shell to the container, monitor its memory and cpu usage, and inspect its configuration.

Portainer is really helpful to manage images and containers that are already on the duckiebot, but what about if you want to create a new container or run a new demo. You could still do it from there, but it is not very intuitive. We commonly use the `dt shell`, that you already have installed.

## Starting a demo using the DT shell {status=ready}
In the Duckietown world, demos are containers that contain a set of functionalities ready to work, if the rest of the Duckiebot is set up properly (e.g. dt-car-interface and dt-duckiebot-interface are running). This is also the moment where the work done in [](#rh-calibration) finally pays off. In order for the demo to work nicely, every Duckiebot must have undergone a calibration procedure to account for its motors' and camera's characteristics. In other words, the calibration procedure ensures that every Duckiebot will behave in the same way when it is given the same set of inputs or commands. 
The demos all follow the same workflow, which is described [here](+opmanual_duckiebot#running-demos).


#### Try out the lane-following demo {#exercise:dt-shell-demo}

Let’s now start a lane_following demo. To do so, follow these [instructions](+opmanual_duckiebot#demo-lane-following).

<end/>

After following the instructions completely, you should have run the lane following demo, and seen the visual output of the lane filter node.

In the duckiebot operation manual, you can find the instructions for the other demos. We mainly use the indefinite_navigation one.
