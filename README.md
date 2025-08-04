###############################################
### File Docker Di configurazione

FROM ros:noetic

#### Imposta la variabile d'ambiente non interattiva
ENV DEBIAN_FRONTEND=noninteractive

#### Aggiorna e installa pacchetti di base
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-noetic-ros-core \
    ros-noetic-ros-base \
    && rm -rf /var/lib/apt/lists/*

#### Inizializza rosdep
RUN rosdep init || true
RUN rosdep update

#### Crea una workspace ROS
RUN mkdir -p /root/**nome_progetto_ros**/src

WORKDIR /root/piezo_sensors_ros

#### Inizializza la catkin_ws
RUN cd /root/piezo_sensors_ros && \
    mkdir -p src && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_init_workspace src && catkin_make" && \
    echo "source /root/**nome_progetto_ros**/devel/setup.bash" >> /root/.bashrc


CMD ["bash"]

##############################################

## Lista dei passaggi per creare docker
1. Creare un cartella container, es. "ros_container"
2. Creare un file Dockerfile di configurazione come quello sopra
3. Fare il build del docker con *docker build -t **nome_progetto_ros***
4. Lanciare il docker con le varie opzioni desiderate:
   * *docker run -it  /* # Avvia un nuovo container interattivo + terminale.
   * *-e DISPLAY=$DISPLAY /*  # per visualizzare i plot.
   * *-v /tmp/.X11-unix:/tmp/.X11-unix /* # per visualizzare i plot.
   * *-v "/c/Users/chris/OneDrive - unige.it/Università/Cosmic/Acquisizione dati ROS/Only_Piezo_ROS:/root/**nome_docker**/src/**nome cartella package**/" /*   # per fare Mount di una directory del tuo host (Windows) nel container.
   * *--network host /*  # Il container usa la rete dell’host direttamente.
   * *--device=/dev/ttyUSB0 /*  # Collega il dispositivo seriale /dev/ttyUSB0 dal tuo host al container.
   * *--name **nome_docker** /*  # Dai un nome al container: piezo_sensor_docker.
   * ***nome prgetto ros** bash  # Nome dell'immagine docker da cui creare il container
5. Dentro il docker:
   * *cd ~/piezo_ros/src*
   * *catkin_create_pkg **nome cartella package** std_msgs rospy roscpp*
   * *cd ~/piezo_ros*
   * *catkin_make*
   * *source devel/setup.bash*



