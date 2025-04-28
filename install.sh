#!/bin/bash

echo "This is the installation script for the project."
echo "It will install the required dependencies and set up the project."
echo "Please make sure you have Python 3.8 or higher installed."
echo "Now let's get started!"

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

# Clone submodules
echo "Cloning git submodules..."
git submodule update --init --recursive
echo "Submodules cloned successfully."

# Create data directory if it doesn't exist
mkdir -p ./cr_vision/data

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

# Colcon build and source
echo "Building the workspace..."
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "Building the workspace done."

echo "Now sourcing the workspace..."
source install/setup.bash
echo "Sourcing finished."

echo "COMPLETE! NOW YOU CAN RUN THE PROJECT. ENJOY! ;)"
