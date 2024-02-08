# https://docs.nvidia.com/jetson/archives/r35.1/DeveloperGuide/text/HR/ControllerAreaNetworkCan.html?highlight=can%20bus#important-features

# sudo apt-get install busybox

sudo busybox devmem 0x0c303018 w 0xc458
sudo busybox devmem 0x0c303010 w 0xc400
sudo busybox devmem 0x0c303008 w 0xc458
sudo busybox devmem 0x0c303000 w 0xc400

sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

sudo ip link set can0 type can bitrate 1000000
sudo ifconfig can0 up
sudo ifconfig can0 txqueuelen 1000

sudo ip link set can1 type can bitrate 1000000
sudo ifconfig can1 up
sudo ifconfig can1 txqueuelen 1000
