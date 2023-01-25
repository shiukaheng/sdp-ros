# 🐢 SDP ROS Turtlebot Template

This is a template repository for using Turtlebots and ROS. It contains a setup script that sets up all the boilerplate to get your Turtlebot up and running.

## Usage
`NOTE: The install script is only meant to work on DICE computers`
- Click "Use this template" on Github, which will create a new repository based on this repository on your account
- Clone your own repository and open a terminal in this repository
- Run `./install.sh` - it sets up a catkin workspace, installs dependencies, and sets up bashrc on both DICE and the Turtlebot
- After installation, you can quickly ssh into your Turtlebot with macro `turtleshell`
- You can then directly develop your packages in your repository, which already has gitignore configured to not commit builds and dependencies
