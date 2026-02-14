# coverage_control

## Requirements
- Ubuntu22.04
- ROS2 Humble
- Python3.10

## Installation
```sh
cd ~/ros2_ws/src/coverage_control
python3 -m pip install -r requirements/lib.txt
rosdep install -i -y --from-paths .
```

```sh
sudo python -m pip install -r requirements/tools.txt
```


## Usage
### simple coverage control
You can select the following parameter through the launch argument:
- dimension of the field: {1,2,3}
- number of agents: {1,2,3,4,5}
- initial density function(phi) type: {1: Uniform, 2: Gaussian, 3: Disk}

```sh
ros2 launch coverage_control scc.launch dim:={1,2,3} num:={1,2,3,4,5} phi:={1,2,3}
```

#### rviz
By adjusting the launch files and the checkboxes in the rviz panels, the density map(phi) and the sensing region(sr) of each agent can be drawn as `Marker` and `Pointcloud2` respectively, as shown below.

<img src=assets/scc_1d_3.png width=50%><img src=assets/scc_1d_5_phi.png width=50%>
<img src=assets/scc_2d_5.png width=50%><img src=assets/scc_2d_5_phi.png width=50%>
<img src=assets/scc_3d_5.png width=50%><img src=assets/scc_3d_4_phi.png width=50%>

#### rqt_graph
<img src=assets/scc_2d_rosgraph.png width=100%>

### persistent coverage control(pcc-devel branch)
```sh
ros2 launch coverage_control pcc.launch dim:={1,2,3} agent_num:={1,2,3,4,5}
```

#### rviz
<img src=assets/pcc_2d_5.gif width=70%>
<img src=assets/pcc_3d_5.gif width=70%>

## Tools
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

## Run On macOS With Docker (Auto GUI via Browser)
### 1. Prerequisites
- Docker Desktop for Mac

### 2. Run natively on Apple Silicon (`linux/arm64`)
```sh
cd coverage_control/docker
DOCKER_PLATFORM=linux/arm64 docker compose -f compose.mac.yaml up --build
```

### 3. Emulate x86_64 on Apple Silicon (`linux/amd64`)
```sh
cd coverage_control/docker
DOCKER_PLATFORM=linux/amd64 docker compose -f compose.mac.yaml up --build
```

### 4. Open GUI automatically
After startup, open:
- `http://localhost:6080/vnc.html`

`rviz2` is launched inside a virtual desktop and streamed by noVNC.

### 4. Optional: rviz2 GUI forwarding on macOS
```sh
# host side
xhost +localhost
```

Then add `DISPLAY=host.docker.internal:0` to the compose service environment.

### Troubleshooting
If launch succeeds but agents stop, check logs:
```sh
docker compose -f compose.mac.yaml logs --tail=200
```

Known issues:
- `AttributeError: np.maximum_sctype was removed ...`
  - Cause: `transforms3d` from ROS2 Humble is not compatible with NumPy 2.x.
  - Fix: this repository pins NumPy to `<1.24` in `requirements/lib.txt`. Rebuild image after pull/update.

Rebuild after dependency changes:
```sh
docker compose -f compose.mac.yaml down
DOCKER_PLATFORM=linux/amd64 docker compose -f compose.mac.yaml up --build
```
