###############################################
#### #Docker Configuration File

FROM ros:noetic

#### #Set non-interactive environment variable
ENV DEBIAN_FRONTEND=noninteractive

#### #Update and install basic packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-tk \
    ros-noetic-ros-core \
    ros-noetic-ros-base \
 && rm -rf /var/lib/apt/lists/*

#### Install Python dependencies with pip
RUN pip install --no-cache-dir matplotlib pyserial numpy

#### #Initialize rosdep
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init && rosdep update
 
#### #Create a ROS workspace
RUN mkdir -p /root/**project_name_ros**/src

#### #Set working directory
WORKDIR /root/**project_name_ros**

#### #Initialize catkin workspace
RUN cd /root/**project_name_ros** && mkdir -p src && /bin/bash -c "source /opt/ros/noetic/setup.bash &&  catkin_init_workspace src && catkin_make" 

#### #Auto-source ROS environment at startup
RUN echo 'source /root/**project_name_ros**/devel/setup.bash' >> /root/.bashrc

####  #Default command
CMD ["bash"]


##############################################

## Steps to build and run Docker
1. Create a folder for the container, e.g. ros_container
2. Enter in the folder:
```bash
cd ros_container
```
4. Create a configuration file **Dockerfile** as shown above inside the folder:
```bash
touch Dockerfile
```
5. Fare il build del docker con:
```bash
 sudo docker build -t **project_name_ros** .
```
6. Run the docker with the desidered options:
   ```bash
   # Run a new Docker container interactively with a terminal
sudo docker run -it \
    -e DISPLAY=$DISPLAY \       # Allow GUI applications to display plots
    -v /tmp/.X11-unix:/tmp/.X11-unix \  # Mount X11 socket to display plots
    --network host \            # Use the host's network directly
    --device=/dev/ttyUSB0 \     # Connect the host's serial device /dev/ttyUSB0
    --name nome_docker \        # Name the container
    nome_progetto_ros bash      # Image to create the container from
```
6. Dentro il docker:
   * *cd ~/**nome_progetto_ros**/src*
   * 
   * *catkin_create_pkg **nome_cartella_package** std_msgs rospy roscpp*
   * *cd ~/**nome_progetto_ros***
   * *catkin_make*
   * *source devel/setup.bash*

## Comandi per lanciare docker esistente ed eliminare docker
* *sudo docker ps -a*  # per vedere docker esistenti
* *sudo docker start -ai **nome_docker***  # per lanciare in esecuzione il docker
* *sudo rm **nome_docker***  # per rimuovere un docker
* *docker exec -it **nome_docker** bash*  # per lanciare un altro terminale collegato al docker

## Lanciare in esecuzione nodi ROS
* Eseguire *roscore* in un terminale del docker
* Eseguire il launch in un altro terminale: *roslaunch **nome_cartella_package** **nome_file.launch***
* Per lanciare un servizio *rosservice call /**nome_servizio.srv**

## Risolvere problemi legati alla GUI
* *xhost +SI:localuser:root* # abilitare il root a usare serve X
* *mkdir -p /tmp/ros_home/.ros* # dentro al docker creare questa cartella

## Creare un'immagine di un docker
* *docker commit **nome_vecchio_docker** **nome_immagine_nuova***
* Fare il run come sopra ma utilizzando la nuova immagine **nome_immagine_nuova** al posto di **nome_immagine_docker**


