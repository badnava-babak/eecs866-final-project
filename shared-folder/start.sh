#!/bin/bash
  

Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
# Continue build instructions from https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md
./waf distclean
./waf configure --board sitl
./waf copter
#RUN ./waf rover
#RUN ./waf plane
#RUN ./waf sub


sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml -y
pip3 install PyYAML mavproxy
echo "export PATH=$PATH:usr/local/bin:/ardupilot/Tools/autotest:/usr/lib/ccache" >> ~/.bashrc
#RUN export PATH=$PATH:$HOME/.local/bin:/ardupilot/Tools/autotest:/usr/lib/ccache



sudo apt install net-tools -y
sudo apt install iputils-ping -y
sudo apt install iproute2 -y
sudo apt update
sudo apt install curl -y
sudo apt install telnet -y

cd "/ardupilot/Tools/autotest"
# CMD /bin/bash -c "source ~/.bashrc"
# Finally the command
./sim_vehicle.py --vehicle ArduCopter --mavproxy-args --out=tcpin:0.0.0.0:5770 --out=udpin:0.0.0.0:14560
