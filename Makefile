all:
	g++ *.cpp -shared -fPIC -I/usr/include/python3.6 -I/usr/lib/x86_64-linux-gnu/ `pkg-config --cflags opencv --libs opencv` -lm -std=c++11 -lboost_system -lboost_python3 -lpython3.6m -o libgraph_canny_segm.so

debug:
	g++ *.cpp -I/usr/include/python3.6 -I/usr/lib/x86_64-linux-gnu/ `pkg-config --cflags opencv --libs opencv` -lm -std=c++11 -lboost_system -lboost_python3 -lpython3.6m -o graph_canny_segm.d.so	