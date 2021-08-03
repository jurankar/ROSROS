echo "export GAZEBO_RESOURCE_PATH=$(pwd):/usr/share/gazebo-11:$GAZEBO_RESOURCE_PATH">> ~/.bashrc

echo "export GAZEBO_MODEL_PATH=$(pwd)/models:$(pwd)/meshes:$GAZEBO_MODEL_PATH">> ~/.bashrc
