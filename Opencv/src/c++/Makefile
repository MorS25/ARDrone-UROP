CC=g++
CFLAGS=  -c -Wall -I/usr/local/include/pcl-1.7/ -I/usr/include/eigen3/ -I/usr/include/vtk-5.10/ -I/usr/include/ssba/ -Wno-deprecated
LDFLAGS= -L/usr/lib/vtk-5.10/ $(shell pkg-config --libs opencv) -lm -lboost_system -lboost_thread -lvtkCommon -lvtkFiltering -lvtkRendering -lpcl_visualization -lpcl_common -lV3D -lcolamd
SRC=video_processing.cpp
SRC_EXE=main.cpp
EXE=main 
OBJ=$(SRC:.cpp=.o)
OBJ_EXE=$(SRC_EXE:.cpp=.o)

main: main.o video_processing.o  
	$(CC) $^ -o $@ $(LDFLAGS)

.cpp.o:
	$(CC) $< -o $@ $(CFLAGS)

clean:
	rm -rf $(OBJ) $(OBJ_EXE) $(EXE)
