sudo bash -c "echo 'deb https://repo.download.nvidia.com/jetson/rt-kernel r32.6 main' >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list"
sudo apt update
sudo apt install nvidia-l4t-rt-kernel nvidia-l4t-rt-kernel-headers