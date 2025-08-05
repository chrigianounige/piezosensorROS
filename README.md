###############################################
#### File Docker Di configurazione

FROM ros:noetic

#### Imposta la variabile d'ambiente non interattiva
ENV DEBIAN_FRONTEND=noninteractive

#### Aggiorna e installa pacchetti di base
RUN apt-get update && apt-get install -y python3-pip python3-rosdep python3-colcon-common-extensions python3-tk ros-noetic-ros-core ros-noetic-ros-base && rm -rf /var/lib/apt/lists/*

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
   * *sudo docker run -it  \* # Avvia un nuovo container interattivo + terminale.
   * *-e DISPLAY=$DISPLAY \*  # per visualizzare i plot.
   * *-v /tmp/.X11-unix:/tmp/.X11-unix \* # per visualizzare i plot.
   * *--network host \*  # Il container usa la rete dell’host direttamente.
   * *--device=/dev/ttyUSB0 \*  # Collega il dispositivo seriale /dev/ttyUSB0 dal tuo host al container.
   * *--name **nome_docker** \*  # Dai un nome al container.
   * ***nome_immagine_docker** bash*  # Nome dell'immagine docker da cui creare il container
5. Dentro il docker:
   * *cd ~/**nome_progetto_ros**/src*
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

## Risolvere problemi legati alla GUI
* *xhost +SI:localuser:root* # abilitare il root a usare serve X
* *mkdir -p /tmp/ros_home/.ros* # dentro al docker creare questa cartella

## Creare un'immagine di un docker
* *docker commit **nome_vecchio_docker** **nome_immagine_nuova***
* 



