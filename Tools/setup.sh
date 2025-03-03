##
# This script sets up the gcl-pendulum environment
##

# Update package lists and upgrade installed packages
sudo apt update
sudo apt upgrade -y

# Install and enable SSH server
sudo apt install -y openssh-server
sudo systemctl enable ssh

# Install net-tools for the "ifconfig" command
sudo apt install -y net-tools

# Install g++ and make
sudo apt install -y g++ make

# Install json for C++ package
sudo apt install -y nlohmann-json3-dev

# Install pigpio
cd ~/
git clone https://github.com/joan2937/pigpio.git
cd pigpio
make
sudo apt install -y python3-distutils
sudo make install
# set pigpio service
cd ~/gcl-pendulum
sudo cp Tools/pigpiod.service /etc/systemd/system/pigpiod.service
sudo systemctl daemon-reload
sudo systemctl enable pigpiod.service


# move pendulum_auto_start_scipt.py
cd ~/
sudo mkdir -p /usr/local/bin/gcl-pendulum
sudo cp ~/gcl-pendulum/Tools/pendulum_auto_start_script.py /usr/local/bin/gcl-pendulum


# Set auto-start script
cd ~/gcl-pendulum
sudo cp Tools/pendulum.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable pendulum.service

# sudo systemctl start pendulum.service
# sudo systemctl status pendulum.service


# Compile
sudo mkdir -p /usr/local/bin/gcl-pendulum
sudo g++ -std=c++17 -o /usr/local/bin/gcl-pendulum/PENDULUM ~/gcl-pendulum/src/*.cpp -lpigpiod_if2 -lrt -pthread
sudo g++ -o /usr/local/bin/gcl-pendulum/PENDULUM_CLEANUP ~/gcl-pendulum/Tools/cleanup.cpp -lpigpiod_if2 -lrt

# Set gain.json
cd ~/gcl-pendulum
mkdir -p param
cp sample/gain.json param/gain.json
