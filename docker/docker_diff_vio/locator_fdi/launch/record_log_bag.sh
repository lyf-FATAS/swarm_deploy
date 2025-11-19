#!/bin/bash
mkdir -p ${HOME}/output
rosbag record /locator_fdi/status -o ${HOME}/output/locator_fdi_log.bag
