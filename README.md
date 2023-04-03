# NA568_team_03_ORBSLAM_LiDAR
NA 568 WN 23 Team 03 Final Project

How to compile:

In CMakeLists.txt:
	Change lines 10 and 11 to wherever you have ORB_SLAM3 and litamin installed
	Change lines 160 and 161 to wherever you have libDBoW2.so and libg2o.so located. These may only appear after you build ORBSLAM3.
	Modify executable to what you want TODO for later once we have code set up

To build and compile, use the following commands inside the main directory:
	mkdir build
	cd build
	cmake ..
	make -j4

Files are output to the out/ folder in the main directory