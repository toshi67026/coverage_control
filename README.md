# coverage_control

## Requirements
- Ubuntu22.04
- ROS2 Humble
- Python3.10

## Installation
```sh
cd ~/ros2/src/coverage_control
python3 -m pip install -r requirements/lib.txt
rosdep install -i -y --from-paths .
```


## Usage
### simple coverage control
TODO: launch argumentからの次元切り替え
```sh
ros2 launch coverage_control scc.launch
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

## Tools
```sh
sudo python -m pip install -r requirements/tools.txt
```

### Format
- isort
- black
```sh
task fmt
```

### Lint
- black
- ruff
```sh
task lint
```

### mypy
- mypy
```sh
task mypy
```
