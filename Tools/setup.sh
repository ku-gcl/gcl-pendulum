##
# This code is for setting up gcl-pendulum environment
##


# update
sudo apt update
sudo apt upgrade

# install and enable ssh
sudo apt install openssh-server
sudo systemctl enable ssh

# install net-tools to do "ifconfig" command
sudo apt install net-tools

# install g++
sudo apt install g++ make


# install pigpio
cd ~/
git clone https://github.com/joan2937/pigpio.git
cd pigpio
make
sudo apt install python3-distutils
sudo make install


