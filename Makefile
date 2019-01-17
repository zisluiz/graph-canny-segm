all:
	g++ *.cpp `pkg-config --cflags opencv --libs opencv` -I/usr/include/python3.7m -lm -std=c++11 -lpython3.7m -lboost_system -lboost_python3 -o graph-canny-segm

