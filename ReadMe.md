# Docker setup for Livox Mid360 LiDAR on Raspberry Pi 5

This document guides you through setting up and running the Livox Mid360 LiDAR driver inside a Docker container on a Raspberry Pi 5.

---

## Prerequisites

* Docker and Docker Compose installed on the host. Be sure to be on the same subnet as the lidar (24). Lidar's ip is 192.168.1.3

---

## 1. Initial Setup

1. Clone this repository and navigate into it.

2. Make the setup script executable and run it:

   ```bash
   chmod +x initial_setup.sh
   ./initial_setup.sh
   ```

   This will install any required dependencies on the host.

3. Place your `msg_MID360_launch.py` configuration file into the `livox_ros_driver2` package (or adjust paths accordingly).

---

## 2. Running the Container

### Launch Docker

Start the container in detached mode:

```bash
docker compose up
```

### Stop Docker

Shut down and remove the container:

```bash
docker compose down
```

### Access the Container Shell

Open an interactive bash session inside the running container:

```bash
docker exec -it docker_driver-livox-1 bash
```

---

## 3. Inside the Container

### Full stack launch

```bash
source install/setup.bash
ros2 launch livox_ros_driver2 stack_launch.py

### Build & Source ROS 2 Workspace

### Launch odometry feed

```bash
cd ../dev_ws
source install/setup.bash
ros2 run py_avoid odom
```

Once inside:

### Launch the Livox Driver

```bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```



### Launch Odometry & Mapping

```bash
source install/setup.bash
ros2 launch super_odometry livox_mid360.launch.py
```

## 3. Recording data and tmux

### Creating a tmux session

```bash
tmux new -s rec
```

In this bash, once recording started : Ctrl-B and D, to close

### To reopen session later

```bash
tmux attach -t rec
```

### To kill tmux

```bash
tmux kill-session -t rec
```

## 4. Next Steps

* Integrate obstacle avoidance.
* Implement high-level pathfinding and planning.
* Tune parameters to your environment and use case.

---

*For more details or troubleshooting, refer to the official Livox ROS Driver documentation or open an issue in this repository.*
