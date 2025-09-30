# Docker Configuration File

```dockerfile
FROM ros:noetic

# Set non-interactive environment variable
ENV DEBIAN_FRONTEND=noninteractive

# Update and install basic packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-tk \
    ros-noetic-ros-core \
    ros-noetic-ros-base \
    git \
    nano \
 && rm -rf /var/lib/apt/lists/*

# Install Python dependencies with pip
RUN pip install --no-cache-dir matplotlib pyserial numpy

# Initialize rosdep
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init && rosdep update

# Create a ROS workspace
RUN mkdir -p /root/**project_name_ros**/src
RUN mkdir -p /tmp/ros_home/.ros*

# Set working directory
WORKDIR /root/**project_name_ros**

# Initialize catkin workspace
RUN cd /root/**project_name_ros** && mkdir -p src && /bin/bash -c "source /opt/ros/noetic/setup.bash &&  catkin_init_workspace src && catkin_make"

# Auto-source ROS environment at startup
RUN echo 'source /root/**project_name_ros**/devel/setup.bash' >> /root/.bashrc

# Default command
CMD ["bash"]
```

# Steps to build and run Docker
1. Create a folder for the container, e.g. ros_container
2. Enter in the folder:
```bash
cd ros_container
```
3. Create a configuration file **Dockerfile**  inside the folder:
```bash
touch Dockerfile
```
4. Copy the example above inside the file
5. Build the docker:
```bash
 sudo docker build -t **project_name_ros** .
```
6. Run the docker with the desidered options:
```bash
# Run a new Docker container interactively with a terminal
# Allow GUI applications to display plots
# Mount X11 socket to display plots
# Use the host's network directly
# Connect the host's serial device /dev/ttyUSB0
# Image to create the container from
sudo docker run -it \
-e DISPLAY=$DISPLAY \ # If you to forward to a other PC connected via ssh you have to put -e DISPLAY=IPV4ADDRESSPC:0       
-v /tmp/.X11-unix:/tmp/.X11-unix \  
--network host \            
--device=/dev/ttyUSB0 \     
--name **container_name** \       
**project_name_ros** bash      
```
7. You are now inside the docker, then:
```bash
   cd ~/**project_name_ros**/src
```
8. You can create a new ROS package or clone from github:
```bash
# Create a new package
catkin_create_pkg **package_name** std_msgs rospy roscpp
```
```bash
# Clone the package from github
git clone **URL**
```
Then:
```bash
cd ~/**nome_progetto_ros**
catkin_make
source devel/setup.bash
```

# Commands to run an existing Docker container and remove Docker containers
```bash
# List all existing Docker containers
sudo docker ps -a
```
```bash
# Start the Docker container interactively
sudo docker start -ai **container_name**
```
```bash
# Remove a Docker container
sudo rm **container_name**
```
```bash
# Open another terminal attached to the Docker container
docker exec -it **container_name** bash
```

# Running ROS nodes
* In one terminal inside the Docker container run:
```bash
roscore
```
* Run the launch file in another terminal:
```bash
cd src/**package_name**
roslaunch **launch_file_name.launch**
```
* To call a service:
```bash
rosservice call /**service_name**
```

# Fixing GUI-related issues
* Outside the docker:
```bash
# Allow root to access the X server
xhost +SI:localuser:root
```
* Inside the container:
```bash
mkdir -p /tmp/ros_home/.ros* 
```

# Creating a Docker image
* Create a new Docker image from an existing container. This saves the current state of the container so you can reuse it later:
```bash
docker commit **old_container_name** **new_image_name**
```
* Run the docker using *docker run ...* as before, but use the new image **new_image_name** instead of **docker_image_name**
This allows you to start a new container with all the changes and setups you made in the original container.



