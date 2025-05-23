#!/bin/bash
cd ../src
echo "This is the installation script for the project."
echo "It will install the required dependencies and set up the project."
echo "Please make sure you have Python 3.8 or higher installed."
echo "Now let's get started!"


COPPELIASIM_RELEASE="V4_10_0_rev0"
COPPELIASIM_FILE="CoppeliaSim_Edu_${COPPELIASIM_RELEASE}_Ubuntu22_04.tar.xz"
if [ ! -f "download/${COPPELIASIM_FILE}" ]; then
    if [ "$1" = "-d" ]; then
        if ! command -v curl > /dev/null 2>&1; then
            echo "Command 'curl' not available" 1>&2
            exit 1
        fi

        echo "Creating download directories..."
        mkdir -p download/tmp

        echo "Downloading ${COPPELIASIM_FILE}"
        cd download/tmp || exit 1
        curl --progress-bar --remote-name --location \
            https://downloads.coppeliarobotics.com/${COPPELIASIM_RELEASE}/${COPPELIASIM_FILE} || exit 1
        mv ${COPPELIASIM_FILE} ..  # move to download/
        cd - || exit 1
    else
        echo "File 'download/${COPPELIASIM_FILE}' not found."
        echo "Either download it manually, or pass the -d option."
        echo "Example: ./install.sh -d"
        exit 1
    fi
fi

tar -xf download/${COPPELIASIM_FILE} -C download
cp -r download/CoppeliaSim_Edu_${COPPELIASIM_RELEASE}_Ubuntu22_04/programming/ros2_packages/sim_ros2_interface ./
rm -rf download

echo "Cartella sim_ros2_interface copiata e tutto il resto eliminato."

# Function -------------------------------------------------------------------------------
check_download() {
    local url="$1"
    local output_path="$2"
    local filename
    filename=$(basename "$output_path")

    if [ -f "$output_path" ]; then
        echo "The file $filename is already downloaded. Skipping this part."
    else
        echo "Downloading $filename..."
        wget --progress=bar:force:noscroll -O "$output_path" "$url" 2>&1 | grep --line-buffered "%" || true
    fi
}

# ----------------------------------------------------------------------------------------


# ----------------------------------------------------------------------------------------
# Clone REPO and submodule
git clone --branch humble https://github.com/PickNikRobotics/ros2_robotiq_gripper.git

mv ros2_robotiq_gripper/robotiq_description ./
rm -rf ros2_robotiq_gripper

git clone --branch humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git ur_description

# Clone submodules
echo "Cloning git submodules..."
git submodule update --init --recursive
echo "Submodules cloned successfully."
# ----------------------------------------------------------------------------------------


# Create data directory if it doesn't exist
mkdir -p /cr_vision/data

# Download YOLO model if not exists
check_download "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-pose.pt" "./cr_vision/data/yolo11n-pose.pt"

# Ask the user about copying or training YOLO model
echo "Do you want to copy the YOLO model for object detection? (y/n)"
read -r user_input

if [ "$user_input" = "y" ]; then
    echo "Copying yolo_cubi.pt to ./cr_vision/data..."
    cp ./yolo_pipline_customdata_basic/yolo_cubi.pt ./cr_vision/data/yolo_cubi.pt
    echo "File copied successfully."
else
    echo "Running training for YOLO models..."
    pip install notebook ultralytics pyyaml
    cd yolo_pipline_customdata_basic
    python3 python_train_yolo_model.py
    echo "Copying trained model (best.pt) to ./cr_vision/data as yolo_cubi.pt..."
    cd ..
    cp ./yolo_pipline_customdata_basic/runs/train/weights/best.pt ./cr_vision/data/yolo_cubi.pt
    echo "Trained model copied successfully."
fi


# # Install dependencies
# echo "Installing dependencies..."
# pip install pyserial
# pip install flask
# pip install flask-ask-sdk
# pip install ask-sdk
# echo "all dependencise are installated"


# # Colcon build and source
# echo "Building the workspace..."
# cd ..
# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
# echo "Building the workspace done."

# echo "Now sourcing the workspace..."
# source install/setup.bash
# echo "Sourcing finished."

echo "COMPLETE! NOW YOU CAN RUN THE PROJECT. ENJOY! ;)"



