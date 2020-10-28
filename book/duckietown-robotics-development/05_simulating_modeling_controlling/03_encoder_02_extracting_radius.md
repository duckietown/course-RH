
## Extracting Model Parameters {status=ready}

Now that we know how to work with the information from the wheel encoders, it is time to make something useful out of them. The task for now will be to implement some calibration functions with the encoders.

To calculate the distance from the wheel encoders, we might use the following approach.

$$ \Delta X = 2*\pi * R * {N_{ticks}/N_{total}} $$

  - $$ \Delta X $$ is the distance travelled per wheel
  - $$ N_{ticks} $$ is the number of ticks measured
  - $$ N_{total} $$ is the number of ticks in one full revolution (e.g. 135).

In Duckietown, the Duckiebots are modeled using a [differential drive](#duckiebot-modeling) model, which depends on several parameters such as the baseline (or distance between the two wheels of the robot), and the wheel radius. To simplify our lives, we assume these are constant values, and we control all Duckiebots with the same parameters. Nevertheless, in the real world this is not often the case, as you have seen already. To overcome this modeling limitation we usually perform wheel calibration, where we manually update some parameters from our configuration files (such as the trim). While this helps to solve individual motor differences, it can still be improved by using the wheel encoders.

We will now look into a different approach. We can also use the wheel encoders to obtain a more accurate model, which extracts the parameters for your own Duckiebot! We will be updating the `baseline` and `radius` parameter from the [kinematics_node.py](https://github.com/duckietown/dt-car-interface/blob/daffy/packages/dagu_car/src/kinematics_node.py). Note that all these parameters can be modified using the `rosparam set` commands.

#### Updating model parameters - Radius {#exercise:extracting-radius}
Do the following:

- Create a copy of the Duckietown ROS template or use your template from the previous exercise.

- Run Keyboard control and manually control your Duckiebot.

- Run your Duckiebot on a straight line for a fixed length (e.g. 1m, or 1 tile) and extract the value of wheel radius. (Note, you might get slightly different values for each wheel, so take the average).

- Use `rosparam set` commands to update the radius parameter in your kinematics configuration file.

Remark: the wheel radius can be found directly from the following formula.
$$ \Delta X = 2*\pi * R * {N_{ticks}/N_{total}} $$

Deliverable: new radius value obtained. Can also be found in `/data/config/calibrations/kinematics/HOSTNAME.yaml`.
<end/>
