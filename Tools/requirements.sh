# This script installs the required packages for the gcl-pendulum.
# This script is for Ubuntu 20.04 on Raspberry Pi 4B.

sudo apt-get update

# Raspberry Piでは、pipでpandasをインストールすると非常に時間がかかるので、apt-getでインストールする
sudo apt-get install -y python3-numpy python3-matplotlib python3-pandas

sudo apt install -y python3-pip
pip3 install control