# Xiao_Setup

## Current State of Code

1. `main.cpp`: Sets up a webserver using a phone hotspot that can be visited to stream ESP cam. 
2. `apriltag.cpp`: Attempts april tag detection using the `esp_camera.h` library (cloned to `/lib`). Currently the code either has false positive detection (pretty arbitrary) or does not detect at all. 
4. `view_cam.py`: Helper script to read and display camera buffers over serial using OpenCV. I can't remember if I got this working or not. 

This code could be easily refactored and improved, it's pretty botched right now.
