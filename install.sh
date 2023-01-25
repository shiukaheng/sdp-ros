# Check if /opt/ros/noetic/setup.bash exists, if it doesn't, ask user to install ROS Noetic and exit
UNDERLAY_SETUP='source /opt/ros/noetic/setup.bash'
if [ ! -d "$UNDERLAY_SETUP" ]; then
  echo "ROS Noetic is not installed, please install ROS Noetic and try again (or just use a DICE machine!)"
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

echo "Installation complete!"