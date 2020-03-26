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

Run: ./graph_canny_segm.d.so rgb_file_name depth_file_name showDebug showImages

Run: ./graph_canny_segm.d.so ./images/rgb_00000.png ./images/depth_00000.png true true
Run: ./graph_canny_segm.d.so ./images/00490-color.png ./images/00490-depth.png true true
Run: ./graph_canny_segm.d.so ./images/000310001400101.jpg ./images/000310001400103.png
Run: ./graph_canny_segm.d.so ./images/rgb_01539.png ./images/depth_01539.png true true
Run: ./graph_canny_segm.d.so ./images/stanley_66_052-image-K-2-1-0.png ./images/stanley_66_052-depth-K-2-1-0.png


Run: ./graph_canny_segm.d.so ./images/000210005140101_rgb.jpg ./images/000210005140101_d.png true true
Run: ./graph_canny_segm.d.so ./images/000310005010101_rgb.jpg ./images/000310005010101_d.png true true

Run: ./graph_canny_segm.d.so ./images/4a7bfe0577f74a1a891683cf5b435f93_52.jpg ./images/4a7bfe0577f74a1a891683cf5b435f93_52.png true true
Run: ./graph_canny_segm.d.so ./images/4d491624b8dd4db9999935affb0c4ada_3.jpg ./images/4d491624b8dd4db9999935affb0c4ada_3.png true true

Run: ./graph_canny_segm.d.so ./images/rgb_000210000020101.png ./images/depth_000210000020101.png true true
Run: ./graph_canny_segm.d.so ./images/rgb_000210000010101.png ./images/depth_000210000010101.png true true

Run: ./graph_canny_segm.d.so ./images/r_000210000010101.jpg ./images/d_000210000010101.png true true
Run: ./graph_canny_segm.d.so ./images/r_00305.png ./images/d_00305.png true true
Run: ./graph_canny_segm.d.so ./images/r_camera_office_55_domain.png ./images/d_camera_office_14_frame_55_domain.png true true

Tested on Ubuntu 18.04 64bit




