###############################################
#### File Docker Di configurazione

FROM ros:noetic

#### Imposta la variabile d'ambiente non interattiva
ENV DEBIAN_FRONTEND=noninteractive

#### Aggiorna e installa pacchetti di base
RUN apt-get update && apt-get install -y python3-pip python3-rosdep python3-colcon-common-extensions ros-noetic-ros-core ros-noetic-ros-base && rm -rf /var/lib/apt/lists/*

#### Inizializza rosdep
RUN rosdep init || true RUN rosdep update

#### Crea una workspace ROS
RUN mkdir -p /root/**nome_progetto_ros**/src

#### Imposta la directory di lavoro
WORKDIR /root/**nome_progetto_ros**

#### Inizializza la catkin_ws
RUN cd /root/**nome_progetto_ros** && mkdir -p src && /bin/bash -c "source /opt/ros/noetic/setup.bash &&  catkin_init_workspace src && catkin_make" 

#### Sorgente automatico dell'ambiente ROS all'avvio
RUN echo 'source /root/**nome_progetto_ros**/devel/setup.bash' >> /root/.bashrc

#### Comando predefinito
CMD ["bash"]


##############################################

## Lista dei passaggi per creare docker
1. Creare un cartella container, es. "ros_container"
2. Creare un file Dockerfile di configurazione come quello sopra
3. Fare il build del docker con *sudo docker build -t **nome_progetto_ros** .*
4. Lanciare il docker con le varie opzioni desiderate:
   * *sudo docker run -it  /* # Avvia un nuovo container interattivo + terminale.
   * *-e DISPLAY=$DISPLAY /*  # per visualizzare i plot.
   * *-v /tmp/.X11-unix:/tmp/.X11-unix /* # per visualizzare i plot.
   * *-v "/c/Users/chris/OneDrive - unige.it/Università/Cosmic/Acquisizione dati ROS/Only_Piezo_ROS:/root/**nome_progetto_ros**/src/**nome_cartella_package**/" /*   # per fare Mount di una directory del tuo host (Windows) nel container.
   * *--network host /*  # Il container usa la rete dell’host direttamente.
   * *--device=/dev/ttyUSB0 /*  # Collega il dispositivo seriale /dev/ttyUSB0 dal tuo host al container.
   * *--name **nome_docker** /*  # Dai un nome al container: piezo_sensor_docker.
   * ***nome prgetto ros** bash*  # Nome dell'immagine docker da cui creare il container
5. Dentro il docker:
   * *cd ~/**nome_progetto_ros**/src*
   * *catkin_create_pkg **nome_cartella_package** std_msgs rospy roscpp*
   * *cd ~/**nome_progetto_ros***
   * *catkin_make*
   * *source devel/setup.bash*

## Comandi per lanciare docker esistente ed elimniare docker
* *sudo docker ps -a*  # per vedere docker esistenti
* *sudo docker start -ai **nome_docker***  # per lanciare in esecuzione il docker
* *sudo rm **nome_docker***  # per rimuovere un docker



