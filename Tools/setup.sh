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



# Set auto-start script
cd ~/gcl-pendulum
sudo cp Tools/pendulum.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable pendulum.service


# Compile
g++ -std=c++17 -o /home/ubuntu/gcl-pendulum/bin/PENDULUM /home/ubuntu/gcl-pendulum/src/*.cpp -lpigpiod_if2 -lrt -pthread
g++ -o /home/ubuntu/gcl-pendulum/bin/PENDULUM_CLEANUP /home/ubuntu/gcl-pendulum/Tools/cleanup.cpp -lpigpiod_if2 -lrt
