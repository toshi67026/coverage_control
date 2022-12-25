# coverage_control

## Requirements
- Ubuntu20.04
- ROS Noetic
- Python3.8

```sh
cd ~/catkin_ws/src/coverage_control
python3 -m pip install -r requirements.txt
rosdep install -i -y --from-paths .
```

## Usage
### simple coverage control
```sh
roslaunch coverage_control scc.launch dim:={1, 2, 3}
```

#### rviz
By adjusting the launch files and the checkboxes in the rviz panels, the density map and the sensing region of each agent can be drawn as `Marker` and `Pointcloud2` respectively, as shown below.

dim:={1, 2, 3}

<img src=assets/scc_1d.png width=50%>
<img src=assets/scc_2d.png width=50%>
<img src=assets/scc_3d.png width=50%>

#### rqt_graph
2d version
![](assets/scc_2d_rosgraph.png)

## tools
### mypy
```sh
./tools/run_mypy.sh
```

### black
```sh
./tools/run_black.sh
```
