# Check if /opt/ros/noetic/setup.bash exists, if it doesn't, ask user to install ROS Noetic and exit
UNDERLAY_SETUP='/opt/ros/noetic/setup.bash'
if [ ! -f "$UNDERLAY_SETUP" ]; then
  echo "ROS Noetic is not installed. Please install ROS Noetic and try again."
  exit 1
fi

echo "Initializing workspace..."
# Source the ROS distribution from the underlay variable
source "$UNDERLAY_SETUP"
# Installation (bash)
FILE="${HOME}/.bashrc"
# Check if FILE exists, if it doesn't, create it
if [ ! -f "$FILE" ]; then
  touch "$FILE"
fi
LINE="source $UNDERLAY_SETUP"
grep -qF -- "$LINE" "$FILE" || echo "$LINE" >> "$FILE"
# Check if "./catkin_ws" exists, if it doesn't, create it
if [ ! -d "./catkin_ws" ]; then
  mkdir ./catkin_ws
fi
# Source /opt/ros/noetic/setup.bash
source "$UNDERLAY_SETUP"
# CD into it and run catkin_make
cd ./catkin_ws
catkin_make
# Get the relative path of "catkin_ws/devel/setup.bash" and add it to .bashrc
OVERLAY_SETUP="$(realpath ./devel/setup.bash)"
LINE="source $OVERLAY_SETUP"
grep -qF -- "$LINE" "$FILE" || echo "$LINE" >> "$FILE"
# Source the new .bashrc
source "$FILE"

echo "Installing dependencies..."
# Clone the Github repos into the src folder
cd ./src
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone http://github.com/ros-perception/openslam_gmapping.git
git clone http://github.com/ros-perception/slam_gmapping.git
git clone http://github.com/ros-planning/navigation.git
git clone https://github.com/ros-planning/navigation_msgs
git clone http://github.com/ros/geometry2.git
cd ..
# Run catkin_make again
catkin_make

echo "Dependencies installed!"

# Set up the environment variables
# Prompt user for the Turtlebot3 model: 1 for "burger", 2 for "waffle_pi", loop until valid input is given
echo "Please select the Turtlebot3 model:"
echo "1. Burger"
echo "2. Waffle Pi"
read -p "Enter your choice: " choice
while [ "$choice" != "1" ] && [ "$choice" != "2" ]; do
  echo "Invalid input, please try again"
  read -p "Enter your choice: " choice
done
# Set the choice to the appropriate model
if [ "$choice" == "1" ]; then
  choice="burger"
elif [ "$choice" == "2" ]; then
  choice="waffle_pi"
fi

# Prompt user for turtlebot_name
read -p "Enter the name of your turtlebot: " turtlebot_name

LINE="export TURTLEBOT3_MODEL=$choice"
grep -qF -- "$LINE" "$FILE" || echo "$LINE" >> "$FILE"
LINE="export ROS_MASTER_URI=https://$turtlebot_name:11311"
grep -qF -- "$LINE" "$FILE" || echo "$LINE" >> "$FILE"
LINE="export ROS_HOSTNAME=$HOSTNAME"
grep -qF -- "$LINE" "$FILE" || echo "$LINE" >> "$FILE"

echo "Environment variables set!"

# Run the new .bashrc
source "$FILE"

# # Turtlebot3 setup
# echo "Next we will set up the Turtlebot3. Please make sure the Turtlebot3 is turned on and in the same network."

# # SSH into the Turtlebot3 using username "pi" and password "turtlebot" and add the following lines to the .bashrc file:
# # sudo ntpdate extntp0.inf.ed.ac.uk

# LINE="sudo ntpdate extntp0.inf.ed.ac.uk"
# ssh 