FROM ubuntu:20.04

ARG COPTER_TAG=Copter-4.0.7

# install git 
RUN apt-get update && apt-get install -y git

# Now grab ArduPilot from GitHub
RUN git clone https://github.com/ArduPilot/ardupilot.git
WORKDIR ardupilot

# Checkout the latest Copter...
#RUN git checkout ${COPTER_TAG}

# Now start build instructions from http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html
RUN git submodule update --init --recursive

# Trick to get apt-get to not prompt for timezone in tzdata

ENV DEBIAN_FRONTEND=noninteractive

# Need sudo and lsb-release for the installation prerequisites
RUN apt-get install -y sudo lsb-release tzdata

# Need USER set so usermod does not fail...
# Install all prerequisites now

#RUN Tools/environment_install/install-prereqs-ubuntu.sh -y
RUN . ~/.profile
# Continue build instructions from https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md
RUN ./waf distclean
RUN ./waf configure --board sitl
RUN ./waf copter
#RUN ./waf rover
#RUN ./waf plane
#RUN ./waf sub


RUN sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml -y
RUN pip3 install PyYAML mavproxy
RUN echo "export PATH=$PATH:usr/local/bin:/ardupilot/Tools/autotest:/usr/lib/ccache" >> ~/.bashrc
#RUN export PATH=$PATH:$HOME/.local/bin:/ardupilot/Tools/autotest:/usr/lib/ccache

# TCP 5760 is what the sim exposes by default
EXPOSE 5770
EXPOSE 14560

# Variables for simulator
ENV INSTANCE 0
ENV LAT 42.3898
ENV LON -71.1476
ENV ALT 14
ENV DIR 270
ENV MODEL +
ENV SPEEDUP 1
ENV VEHICLE ArduCopter

RUN sudo apt install net-tools -y
RUN sudo apt install iputils-ping -y
RUN sudo apt install iproute2 -y
RUN sudo apt update
RUN sudo apt install curl -y
RUN sudo apt install telnet -y

WORKDIR "/ardupilot/Tools/autotest"
# CMD /bin/bash -c "source ~/.bashrc"
# Finally the command
CMD ["./sim_vehicle.py", "--custom-location=38.95209759531131,-95.26429358856721,0,0", "--vehicle", "ArduCopter", "--mavproxy-args", "--out=tcpin:0.0.0.0:5770", "--out=udpin:0.0.0.0:14560"]


#COPY start.sh /
#RUN  chmod +x /start.sh 

# Copying to /etc/resolv.conf is done inside start.sh
#CMD [ "/start.sh"]

