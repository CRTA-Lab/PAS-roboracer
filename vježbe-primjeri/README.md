# F1TENTH simulator (ROS 2 Jazzy, linux/amd64)

Prebuilt Docker image of [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) (`dev-jazzy` branch)
with RViz in the browser (noVNC) and the `wall_follow` package included.

## Requirements

- Linux on an x86_64 (amd64) machine
- Docker Engine with the `docker compose` plugin: https://docs.docker.com/engine/install/

## 1. Load the image (once)

```bash
docker load -i f1tenth_gym_ros_jazzy_amd64.tar.gz
```

## 2. Start the containers

From this folder:

```bash
docker compose up -d
```

This starts the simulator container and a noVNC container for the display.

## 3. Open the display

Open http://localhost:8080/vnc.html in your browser and click **Connect**.

## 4. Launch the simulator

```bash
docker exec -it f1tenth_jazzy_amd64-sim-1 bash
```

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

RViz opens in the noVNC tab with the map and the car.

## 5. Run wall_follow

In a second terminal (or a new `tmux` window: `Ctrl+b` then `c`):

```bash
docker exec -it f1tenth_jazzy_amd64-sim-1 bash
```

```bash
ros2 run wall_follow wall_follow
```

## Editing your own code

`./student` is mounted at `/sim_ws/src/student` in the container. Edit files on your machine,
then rebuild inside the container:

```bash
cd /sim_ws && colcon build --symlink-install && source install/setup.bash
```

New packages you add to `./student` are picked up the same way.

## Stop

```bash
docker compose down
```

## Notes

- Keyboard teleop is enabled; the sim LiDAR has 1080 beams over 270°.
- The container folder name `f1tenth_jazzy_amd64` sets the container name
  (`f1tenth_jazzy_amd64-sim-1`). If you rename the folder, use `docker ps` to find the new name.
- Foxglove also works instead of RViz: connect to `ws://localhost:8765` at https://app.foxglove.dev.
