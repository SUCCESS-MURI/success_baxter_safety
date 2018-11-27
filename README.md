# success_baxter_safety
License - MIT  
Maintainer - zhi.tan@ri.cmu.edu  

A node that monitors the baxter's endpoint and joint orientations, velocities, and positions. When any safety parameters (as set forth in a yaml file) are violated, the robot E-stops and kills all activity.


## Usage
By default, the system reads the `default_params.yaml` located at the root of this package. You can change/set other yaml parameter files by setting the relative rosparam `safety_param_file_path`.

### Editing Parameters
Set the safety parameters, according the form in `default_params.yaml`. Leave any parameters you don't want to monitor as blank dictionary entries in the file. You can also take out 1st level params (`endpoint_orientation`, `endpoint_position`) if you don't want to monitor them.

### Running Safety Node
```
rosrun success_baxter_safety safety.py [safety_param_file_path:=PATH_TO_FILE]
```
OR
```
roslaunch success_baxter_safety safety.launch [safety_param_file_path:=PATH_TO_FILE]
```

## Past Contributors
- Joe Connolly - 07/2018