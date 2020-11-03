
## Extracting Model Parameters {status=ready}

Now that we know how to work with the information from the wheel encoders, it is time to make something useful out of them. The task for now will be to implement some calibration functions with the encoders.

In Duckietown, Duckiebots are modeled using a [differential drive](https://docs.duckietown.org/DT19/learning_materials/out/duckiebot_modeling.html) model, which depends on several parameters such as the baseline (or distance between the two wheels of the robot), and the wheel radius. To simplify our lives, we assume these are constant values, the same across all Duckiebots. Nevertheless, in the real world this is often not the case, as you have already seen. To overcome this modeling limitation we usually perform wheel calibration, where we manually update some parameters from our configuration files (such as the trim value). While this helps to solve individual motor differences, it can still be improved by using the wheel encoders.

We can use the wheel encoders to obtain an accurate model, which extracts the parameters for your own Duckiebot! We will be updating the `baseline` and `radius` parameters used by the [kinematics_node.py](https://github.com/duckietown/dt-car-interface/blob/daffy/packages/dagu_car/src/kinematics_node.py). Note that all these parameters can be modified using the `rosparam set` commands. However, to make the change permanent by writing it to the corresponding configuration file, you would need to use the `save_calibration` ros service call.

#### Updating model parameters - Radius {#exercise:extracting-radius}
Do the following:

- Create a copy of the Duckietown ROS template or use your template from the previous exercise.

- Run Keyboard control and manually control your Duckiebot.

- Run your Duckiebot on a straight line for a fixed length (e.g. 1m, or 1 tile) and extract the value of the wheel radius. You might get slightly different values for each wheel, so take the average.

- Use `rosparam set` commands to update the radius parameter in your kinematics configuration file.

- After updating the parameters, make the change permanent by calling the `save_calibration` service to save the file: `rosservice call /DUCKIEBOT_NAME/kinematics_node/save_calibration`.

Remark: the wheel radius can be found directly from the following formula.
$$ \Delta X = 2*\pi * R * {N_{ticks}/N_{total}} $$

Deliverable: new radius value obtained. Can also be found in `/data/config/calibrations/kinematics/HOSTNAME.yaml`, after you called the `save_calibration` service.

Note: In order to make the effects of this exercise more pronounced, you can first restore the `trim` value in your `/data/config/calibrations/kinematics/HOSTNAME.yaml` configuration file back to its default value of 0.0.
<end/>
