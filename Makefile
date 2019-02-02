all:
	g++ *.cpp -shared -fPIC `pkg-config --cflags opencv --libs opencv` -lm -std=c++11 -lboost_system -o graph-canny-segm.so

debug:
	g++ *.cpp `pkg-config --cflags opencv --libs opencv` -lm -std=c++11 -lboost_system -o graph-canny-segm.d.so	