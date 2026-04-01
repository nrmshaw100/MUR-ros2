sudo timedatectl set-ntp false

read -p "Input the current date and time (YYYY-MM-DD HH:MM:SS): " datetime
sudo timedatectl set-time "$datetime"

source /home/jetson/ros2_ws/install/setup.bash