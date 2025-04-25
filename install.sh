echo This is the installation script for the project.
echo It will install the required dependencies and set up the project.
echo Please make sure you have Python 3.8 or higher installed.
echo now let-s started:

# function -------------------------------------------------------------------------------
check_download(){
    local url="$1"
    local output_path="$2"
    local filename=$(basename "$output_path")

    if [ -f "$output_path" ]; then
        echo "Il file $filename already downloaded. Skipping this part."
    else
        echo "downlaod $filename..."
        wget --progress=bar:force:noscroll -O "$output_path" "$url" 2>&1 | grep --line-buffered "%" || true
    fi
}


# ---------------------------------------------------------------------------------------



# download_if_not_exists yolo model
mkdir -p ./cr_vision/data

check_download "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11x.pt" "./cr_vision/data/yolo11x.pt"
check_download "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-pose.pt" "./cr_vision/data/yolo11n-pose.pt"


# colcon build and make source
echo "Building the workspace..."
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "Building the workspace done."
echo "Now sorce the workspace..."
source install/setup.bash

