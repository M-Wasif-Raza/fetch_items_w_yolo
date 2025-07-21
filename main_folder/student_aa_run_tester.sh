#! /bin/bash


# To use:
# Download the zip file you submitted on KEATS and the script below.
# Start the itr container.
# Go to the directory where your zip file is.
# Run the script as follows:

# bash student_aa_run_tester.sh <FILE NAME>.zip <VIDEO PATH> <YOLO_COCO_DATA_PATH>

# Where <FILE NAME> is the name of your file (i.e., resit_coursework.zip), 
# and <VIDEO PATH> is the full path to the folder where the videos are located.
# <YOLO_COCO_DATA_PATH> is the full path to the yolo coco data folder.

# Once you run this, a new workspace will be created in /tmp, catkin_make will be run,
# permissions will be set, the workspace will be sourced, and the launchfile will be called.
# If you see Rviz and your code runs without errors, it should also run on our computer. 
# Emphasis is on the should; if something fails in our computer and we cannot run it, we cannot amend this. However, if the script works, this should be a good indicator that the code will also work on our test machine.

VIDEO_PLAYER_NODE="itr_cw_2425_resit"

ZIP_FILE=$1
if [ $# -ne 3 ]; then
    echo "Error: Exactly three arguments are required."
    exit 1
fi

if [[ "$1" != *.zip ]]; then
    echo "Error: The first argument must have a .zip extension."
    exit 1
fi
VIDEO_FOLDER_PATH=$2
YOLO_COCO_DATA=$3

echo "STARTING TESTER SCRIPT ON FILE: $ZIP_FILE . The script requires internet to run!!"

# INITIALISE: CLEAR ROS TERMINAL FROM PREVIOUS RUNS
source /opt/env.sh

mkdir -p /tmp/aa_ws_test/src
cp "$PWD/$ZIP_FILE" /tmp/aa_ws_test/src
cd /tmp/aa_ws_test/src
unzip -o $ZIP_FILE > /dev/null
rm $ZIP_FILE
cd resit_coursework
find . -name "main_node.py" -exec chmod +x {} \;
find . -name "$VIDEO_PLAYER_NODE" -exec rm -f {} \;
wget -q https://nextcloud.nms.kcl.ac.uk/s/Nkds7Ttsj8DmTmC/download -O $VIDEO_PLAYER_NODE --show-progress
if [[ $? -ne 0 ]]; then
    echo "There was an error downloading the video player node. Please check your internet and try again."
    rm -rf /tmp/aa_ws_test
    exit 1;
fi
chmod +x $VIDEO_PLAYER_NODE

# Fix paths for yolo coco path
find . -type f -name "*.py" -exec sed -i "s|meta_path=\(.*\)#.*|meta_path=\1|g" {} \;  # Remove comments
find . -type f -name "*.py" -exec sed -i "s|meta_path=.*(.*)|meta_path='$YOLO_COCO_DATA'|g" {} \; # Remove function calls
find . -type f -name "*.py" -exec sed -i "s|meta_path \?= \?[a-z|A-Z|_]\+|meta_path='$YOLO_COCO_DATA'|g" {} \; # for cases like meta_path = variable
find . -type f -name "*.py" -exec sed -i "s|meta_path=.*['\"]|meta_path='$YOLO_COCO_DATA'|g" {} \;
find . -type f -name "*.py" -exec sed -i "s|meta_path =.*['\"]|meta_path='$YOLO_COCO_DATA'|g" {} \; # Case with a space
##

cd ../..

catkin_make

if [ $? -ne 0 ]; then
    echo "Error: catkin_make failed. The automatic marker wouldn't run!"
    rm -rf /tmp/aa_ws_test
    exit 1
fi

source devel/setup.bash
echo -e "$(wget -qO- https://nextcloud.nms.kcl.ac.uk/s/eCLBr8S6RLEoYa2/download --show-progress | head -n -3)\n</launch>" > _test_cw.launch
roslaunch ./_test_cw.launch video_folder:=$VIDEO_FOLDER_PATH

rm -rf /tmp/aa_ws_test

echo "If you the simulator, rviz launched, and your code did not crash and ran as expected, then it should also run in the marking computer."
echo "If you see any errors, please fix them and resubmit your coursework."