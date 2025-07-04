#!/bin/bash
# LORENZO
source /opt/ros/noetic/setup.bash
# Use GPU
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
# JORGE
source /home/dwa_ws/devel/setup.bash
source /source_px4.sh
exec "$@"  # This will execute the provided command (like /bin/bash)
