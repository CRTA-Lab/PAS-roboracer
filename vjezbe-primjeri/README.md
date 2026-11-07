# F1TENTH simulator (ROS 2 Jazzy, linux/amd64)

Prebuilt Docker image of [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) (`dev-jazzy` branch)
with RViz in the browser (noVNC) and the `wall_follow` package included.

## Requirements

- Linux on an x86_64 (amd64) machine
- Docker Engine with the `docker compose` plugin: https://docs.docker.com/engine/install/

## 1. Start the containers

From this folder, in a terminal run:

```bash
docker compose up -d
```

This starts the simulator container and a noVNC container for the display.

## 2. Launch the sim

In a terminal run:

```bash
docker exec -it roboracer-sim bash
```

You are now inside roboracer-sim docker container. Run the simulator with:

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py config:=/sim_ws/config/sim_config.yaml
```

This passes the parameters located in /vjezbe-primjeri/config/sim_config.yaml on the host PC.

## 3. Open the display

### Foxglove

Download [foxglove studio](https://foxglove.dev/download) and connect to ws://localhost:8765.

### NoVNC

Open http://localhost:8080/vnc.html in your browser and click **Connect**.

## 4. Run wall_follow

In another terminal run:

```bash
docker exec -it roboracer-sim bash
```

```bash
ros2 run wall_follow wall_follow
```

This will run wall_follow example. In a similar manner, you can run your own code that should be located at `/sim_ws/src/student` inside the container.

## Editing your own code

There are a couple of ways to tryout your own code:

### Outside docker

`./student` is mounted at `/sim_ws/src/student` in the roboracer-sim container. Edit files on your machine,
then rebuild inside the container:

```bash
cd /sim_ws && colcon build --symlink-install && source install/setup.bash
```

New packages you add to `./student` are picked up the same way.

### Inside docker with a .devcontainer

With a VSCode devcontainer extension, development can be done directly inside a docker container. 
After downloading the [extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers), Ctrl+Shift+P and select reopen in container. Configuration is already ready to go and located in .devcontainer folder.

## Stop

```bash
docker compose down
```

## Notes

- Keyboard teleop is enabled; the sim LiDAR has 1080 beams over 270°.
- The container folder name `f1tenth_jazzy_amd64` sets the container name
  (`f1tenth_jazzy_amd64-sim-1`). If you rename the folder, use `docker ps` to find the new name.
- Foxglove also works instead of RViz: connect to `ws://localhost:8765` at https://app.foxglove.dev.
