source ./helpers.sh

# Check if /opt/ros/noetic/setup.bash exists, if it doesn't, ask user to install ROS Noetic and exit
UNDERLAY_SETUP='/opt/ros/noetic/setup.bash'
require_file "$UNDERLAY_SETUP" "ROS Noetic is not installed. Please install ROS Noetic and try again."

announce "Initializing workspace..."

# Source the ROS distribution from the underlay variable
source "$UNDERLAY_SETUP"

# Installation (bash)
require_or_create_file "$BASHRC"
write_bashrc "source $UNDERLAY_SETUP"

# Check if "./catkin_ws" exists, if it doesn't, create it
OVERLAY_FOLDER='./catkin_ws'
require_or_create_folder "$OVERLAY_FOLDER"

# CD into it and run catkin_make
cd ./catkin_ws

# Make the src folder if it doesn't exist
require_or_create_folder "./src"
catkin_make

# Get the relative path of "catkin_ws/devel/setup.bash" and add it to .bashrc
OVERLAY_SETUP="$(realpath ./devel/setup.bash)"
require_file "$OVERLAY_SETUP" "catkin_make failed. Please try again."

# Source the new .bashrc
source "$BASHRC"

announce "Installing dependencies..."

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
# Make .dependencies_installed hidden
chmod 600 ".dependencies_installed"
announce "Dependencies installed!"
cd ..

# Set up the environment variables

# Prompt user for the Turtlebot3 model: 1 for "burger", 2 for "waffle_pi", loop until valid input is given
announce "Please select the Turtlebot3 model:"
echo "1. Burger"
echo "2. Waffle Pi"
read -p "Enter your choice: " choice
while [ "$choice" != "1" ] && [ "$choice" != "2" ]; do
  error "Invalid input, please try again"
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

write_bashrc "export TURTLEBOT3_MODEL=$choice"
write_bashrc "export ROS_MASTER_URI=http://$turtlebot_name:11311"
write_bashrc "export TURTLEBOT_NAME=$turtlebot_name"
write_bashrc "export ROS_HOSTNAME=$HOSTNAME"

MACRO_PATH="$(realpath ../macros.sh)"
write_bashrc "source $MACRO_PATH"

announce "Environment variables set!"

# Run the new .bashrc
source "$BASHRC"

# # Turtlebot3 setup
announce "Next we will set up the Turtlebot3. Please make sure the Turtlebot3 is turned on and in the same network. Press any key to continue..."
read -n 1 -s

# Send "install_turtlebot.sh" and "helpers.sh" to $turtlebot_name using scp
announce "Moving install_turtlebot.sh and helpers.sh to $turtlebot_name... Please enter the password for user pi on $turtlebot_name when prompted."
scp *.sh pi@$turtlebot_name:~/
ssh pi@$turtlebot_name "chmod +x install_turtlebot.sh && ./install_turtlebot.sh"

announce "Turtlebot3 setup complete!"
