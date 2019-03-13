## Fast Graph-Based Object Segmentation for RGB-D Images

# Original and forked work

CPU code for paper:
Fast Graph-Based Object Segmentation for RGB-D Images, Giorgio Toscana, Stefano Rosa
https://arxiv.org/abs/1605.03746

# My contributions

* Code updated to date frameworks
* Parameters from config file
* Optional print logs and image show by showDebug and showImages arguments
* Python support with ctypes

# Dependencies:
sudo apt-get install libopencv-dev libboost-system-dev

# Compile: 

For library usage: make 
For tests: make debug

# Tests

Run: ./graph-canny-segm.d.so rgb_file_name depth_file_name showDebug showImages

Run: ./graph-canny-segm.d.so ./images/rgb_00000.png ./images/depth_00000.png true true
Run: ./graph-canny-segm.d.so ./images/00490-color.png ./images/00490-depth.png true true
Run: ./graph-canny-segm.d.so ./images/000310001400101.jpg ./images/000310001400103.png
Run: ./graph-canny-segm.d.so ./images/rgb_01539.png ./images/depth_01539.png
Run: ./graph-canny-segm.d.so ./images/stanley_66_052-image-K-2-1-0.png ./images/stanley_66_052-depth-K-2-1-0.png

Tested on Ubuntu 18.04 64bit




