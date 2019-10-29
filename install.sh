#!/bin/bash
printf "IASlab UNIPD Autonomous Robotics Environment Setup Automatic Routine \n"

kinect_serial_nr=009927144947

# Save the current path
CURRENT_DIR=$(pwd)

# Search for ROS workspace
printf "Please insert the ABSOLUTE path to your ROS workspace (i.e. /home/iaslab/catkin_ws): \n"
read -e -p "Catkin Workspace Path:" ROS_WORKSPACE
printf "Selected Catkin Workspace Path: ${ROS_WORKSPACE} \n"

if [ ! -d "${ROS_WORKSPACE}/src" ]; then
    printf "Selected path does not exists. Do you want to create it? [y/N]: \n"
    read CREATE_WORKSPACE

    if  [ ${CREATE_WORKSPACE} == "y" ]; then
        mkdir -p ${ROS_WORKSPACE}/src/
    fi
fi

if [ -d "${ROS_WORKSPACE}" ]; then
    # Define Ubuntu version and ROS distribution in case of future needs
    UBUNTU_VERSION=bionic
    ROS_DISTRO=melodic

#     # Challenge bash scripts for connecting to the real robots
#     echo -n "Do you want to add the challenge bash script? [y/N]: "
#     read EMARC_SCRIPTS
#     if  [ ${EMARC_SCRIPTS} == "y" ]; then
#         echo ""  >> ~/.bashrc
#         echo "if [ -f ~/.bash_emarc ]; then" >> ~/.bashrc
#         echo "  source ~/.bash_emarc" >> ~/.bashrc
#         echo "fi" >> ~/.bashrc
#     fi

    # Updating Ubuntu APT repositories
    echo "Please insert username and password: "
    read -p 'Username: ' uservar
    read -sp 'Password: ' passvar
    echo ${passvar} | sudo -S apt-get update


    #Moveit and Gazebo ROS standard packages
    sudo apt-get install -y \
        python-catkin-tools \
        ros-${ROS_DISTRO}-controller-manager \
        ros-${ROS_DISTRO}-moveit* \
        ros-${ROS_DISTRO}-gazebo-ros-pkgs \
        ros-${ROS_DISTRO}-gazebo-ros-control \
        ros-${ROS_DISTRO}-joint-state-controller \
        ros-${ROS_DISTRO}-position-controllers \
        ros-${ROS_DISTRO}-velocity-controllers \
        ros-${ROS_DISTRO}-effort-controllers \
        ros-${ROS_DISTRO}-joint-trajectory-controller \
        ros-${ROS_DISTRO}-usb-cam \
        ros-${ROS_DISTRO}-hardware-interface \
        ros-${ROS_DISTRO}-industrial-msgs


    internal_src_path=${ROS_WORKSPACE}/src/internal
    mkdir -p ${internal_src_path}
    tp_src_path=${ROS_WORKSPACE}/src/third_party
    mkdir -p ${tp_src_path}

    # Dowload UR ROS source packages
    echo -n "Do you want to install the UR ROS packages? [y/N]: "
    read enable_ur_pkgs

    if  [ ${enable_ur_pkgs} == "y" ]; then
        cd ${tp_src_path}
        git clone https://github.com/ros-industrial/universal_robot.git --branch melodic-devel
        git clone https://github.com/ros-industrial/ur_modern_driver.git --branch kinetic-devel
    fi

    # Robotiq 3 fingers gripper ROS standard packages
    echo ${passvar} | sudo -S apt-get install -y \
        ros-${ROS_DISTRO}-soem \
        ros-${ROS_DISTRO}-socketcan-interface \
        python-pymodbus

    # Simulation sensors and actuators ROS standard packages
    echo ${passvar} | sudo -S apt-get install  -y \
        ros-${ROS_DISTRO}-openni-launch \
        ros-${ROS_DISTRO}-object-recognition-msgs

    # Simulation sensors and actuators ROS source packages
    echo -n "Do you want to install the Simulation sensors and actuators ROS packages? [y/N]: "
    read enable_simulation_pkgs

    if  [ ${enable_simulation_pkgs} == "y" ]; then
        cd ${tp_src_path}
        git clone https://github.com/JenniferBuehler/general-message-pkgs.git
        git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
        git clone https://github.com/JenniferBuehler/common-sensors.git
    fi


    cd ${tp_src_path}
    # Robotiq Gripper Official Package
    git clone  https://github.com/ros-industrial/robotiq.git --branch kinetic-devel

    # Apriltag Ros Packages
    git clone https://github.com/AprilRobotics/apriltag.git
    git clone https://github.com/AprilRobotics/apriltag_ros.git

    # libfreenect2
    cd $HOME
    mkdir -p ext_libs && cd ext_libs
    sudo apt-get install build-essential\
                         cmake\
                         pkg-config\
                         libusb-1.0-0-dev\
                         libturbojpeg0-dev\
                         libglfw3-dev\
                         libopenni2-dev

    git clone https://github.com/OpenKinect/libfreenect2.git libfreenect2
    cd libfreenect2
    mkdir build && cd build
    cmake .. -DENABLE_CXX11=ON -DCUDA_PROPAGATE_HOST_FLAGS=off -DCMAKE_INSTALL_PREFIX=$HOME/ext_libs/libfreenect2/install
    make install -j$nproc
    sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
    echo "export freenect2_DIR=$HOME/ext_libs/libfreenect2/install/lib/cmake/freenect2" >> $HOME/.bashrc
    source $HOME/.bashrc

    # iai_kinect2
    cd ${tp_src_path}
    git clone https://github.com/code-iai/iai_kinect2.git
    cd iai_kinect2
    rosdep install -r --from-paths .

    cd $HOME/ext_libs/
    mkdir tmp && cd tmp
    git clone https://bitbucket.org/iaslab-unipd/sensor_info.git
    cd sensor_info/kinect2
    cp -r ${kinect_serial_nr} ${tp_src_path}/iai_kinect2/kinect2_bridge/data/
    cd ${tp_src_path}
    rm -rf $HOME/ext_libs/tmp

    # Easy-HandEye
    cd ${tp_src_path}
    git clone https://github.com/IFL-CAMP/easy_handeye
    cd ${ROS_WORKSPACE}
    rosdep install -iyr --from-paths src

    echo 'export LC_NUMERIC="en_US.UTF-8"' >> $HOME/.bashrc
    source $HOME/.bashrc

    # Simulation sensors and actuators ROS source packages
    echo -n "Do you want to install the Unipd Autonomous Robotics ROS packages? [y/N]: "
    read enable_autonomous_robotics_pkgs

#     # Challenge Environment and Robots Configuration ROS source packages
#     if  [ ${CHALLENGE_PKGS} == "y" ]; then
#         # Request for BitBucket information
#         echo -n "Enter your BitBucket username and press [ENTER]: "
#         read BIT_USER

#         # Set git to use the credential memory cache
#         git config --global credential.helper cache
#         # for the selected user
#         git config --global credential.https://github.com.username ${BIT_USER}
#         # Set the cache to timeout after 1 hour (setting is in seconds)
#         git config --global credential.helper 'cache --timeout=3600'

#         # Cloning challenge_arena package...
#         git clone https://${BIT_USER}@bitbucket.org/iaslab-unipd/challenge_arena.git
#         wait
#         sleep 5
#         # Cloning ur10_platform_challenge_moveit_config package...
#         git clone https://${BIT_USER}@bitbucket.org/iaslab-unipd/ur10_platform_challenge_moveit_config.git
#         wait
#         sleep 5
# 	# Cloning the challenge arena collision object package...
#         git clone https://${BIT_USER}@bitbucket.org/iaslab-unipd/arena_collision.git
#         wait
#         sleep 5
#
#         rosdep install robotiq_modbus_tcp

#         wait
#         sleep 5
#         git clone https://github.com/RIVeR-Lab/apriltags_ros.git
#         wait
#         sleep 5
#         git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git
#         wait sleep 5
#         # Cloning marrtino_navigation package...
#         git clone https://${BIT_USER}@bitbucket.org/koide3/marrtino_navigation.git
#         wait
#         sleep 5

#     fi

#     # Compiling
    cd ${ROS_WORKSPACE}
    catkin build -DCMAKE_BUILD_TYPE=Release

#     # Environment setup
    source ${ROS_WORKSPACE}/devel/setup.bash
#     rosrun challenge_arena EMARC_disconnect.sh
fi

# # Go back to the initial directory
cd ${CURRENT_DIR}
