#!/bin/bash

echo "Running path planner on configuration file" "$1"
echo ================================================
cat "$1"
echo ================================================


echo
java -cp bin/:lib/JPathPlan-v1.6.jar:lib/Apparate-v2.8.jar pplanning.simviewer.controller.Launcher "$1"
