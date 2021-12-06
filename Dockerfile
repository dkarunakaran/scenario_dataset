FROM ubuntu:18.04

# Cloning git repo
RUN apt-get update

# Setup git
RUN apt-get install -y git
RUN git config --global user.name "Dhanoop Karunakaran"
RUN git config --global user.email "dkar9051@uni.sydney.edu.au"

# IMPORTANT TO CHANGE DEPENDS ON THE CONFIG YOU HAVE: ADD <ssh private file name> /root/.ssh/id_rsa
ADD .ssh /root/.ssh
RUN touch /root/.ssh/known_hosts

ADD .vimrc /root/.vimrc

# IMPORTANT TO CHANGE DEPENDS ON THE GIT REPO YOU HAVE: RUN ssh-keyscan <git repo domain> >> /root/.ssh/known_hosts
RUN ssh-keyscan gitlab.acfr.usyd.edu.au >> /root/.ssh/known_hosts
RUN apt-get update

# Installing ROS-melodic
RUN apt-get install -y gnupg2
RUN apt-get install -y curl
RUN apt-get install -y lsb-core
ARG DEBIAN_FRONTEND=noninteractive

# INSTALL OTHER NECESSARY PACKAGES
RUN apt-get install -y vim
RUN apt-get install -y wget
RUN apt-get update
RUN apt-get install -y python-pip
RUN apt-get install -y libpng16-16
RUN apt-get install -y libjpeg-turbo8
RUN apt-get install -y libtiff5
RUN apt-get update
RUN pip install tensorflow==2.0.0b0
RUN pip install python-pcl
RUN pip install pyproj
RUN pip install scipy
RUN pip install sklearn

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -
RUN apt update
RUN apt install -y ros-melodic-desktop
RUN apt-get install -y python-rosdep
RUN rosdep init
RUN rosdep update
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
#RUN source ~/.bashrc
RUN apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential

# Intalling pyton-catkin
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get update
RUN apt-get install -y python-catkin-tools
RUN apt-get install -y software-properties-common

# Other important packages
RUN apt-get install -y ros-melodic-tf2-sensor-msgs
RUN apt-get install -y ros-melodic-tf2-geometry-msgs
RUN apt-get install  -y ros-melodic-lanelet2

# Numpy like c++ library
#RUN apt-get install -y xtensor-dev

# Install Carla 0.9.12
RUN cd /opt && wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.12.tar.gz && mkdir carla-simulator && tar -xvf CARLA_0.9.12.tar.gz -C carla-simulator && rm -f CARLA_0.9.12.tar.gz
RUN echo "export PYTHONPATH=$PYTHONPATH:/opt/carla-simulator/PythonAPI/carla/dist/carla-0.9.12-py2.7-linux-x86_64.egg" >> ~/.bashrc

# Install carla-ros-bridge
RUN apt-get update
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1AF1527DE64CB8D9
RUN add-apt-repository "deb [arch=amd64] http://dist.carla.org/carla $(lsb_release -sc) main"
RUN apt-get update
RUN apt-get install -y carla-ros-bridge
RUN echo "source /opt/carla-ros-bridge/melodic/setup.bash" >> ~/.bashrc
RUN source /opt/carla-ros-bridge/melodic/setup.bash

#RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 81061A1A042F527D
#RUN add-apt-repository "deb [arch=amd64 trusted=yes] http://dist.carla.org/carla-ros-bridge-melodic/ bionic main"
#RUN apt update && sudo apt install -y carla-ros-bridge-melodic

#RUN echo "source /opt/carla-ros-bridge/melodic/setup.bash" >> ~/.bash_profile
#RUN echo "source /opt/carla-ros-bridge/melodic/setup.bash" >> ~/.bashrc

RUN pip install networkx
RUN ssh-keyscan github.com >> /root/.ssh/known_hosts
RUN apt-get update

RUN apt-get install -y ros-melodic-jsk-recognition-msgs 
RUN apt-get install -y ros-melodic-jsk-rviz-plugins
RUN apt-get install -y ros-melodic-grid-map
RUN apt-get install -y ros-melodic-audio-common
RUN apt-get install -y libglew-dev
RUN apt-get install -y ros-melodic-geodesy
RUN apt-get install -y freeglut3-dev

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bash_profile

CMD ["tail", "-f", "/dev/null"]
