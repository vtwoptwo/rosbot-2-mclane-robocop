#!/usr/bin/bash

docker compose down

export ROS_MASTER_URI=http://10.205.3.24:11311
export ROS_IP=10.205.3.24


echo "Set master to $ROS_MASTER_URI"
echo "Set ip to $ROS_IP"

echo "Setting up docker"

docker compose up -d
