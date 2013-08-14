CC=gcc
CFLAGS=  -c -Wall
LDFLAGS=$(shell pkg-config --libs opencv) -lm
SRC=video_processing2.c video_processing3.c 
SRC_EXE=main.c mainSurf.c
EXE=main 
OBJ=$(SRC:.c=.o)
OBJ_EXE=$(SRC_EXE:.c=.o)

main: mainSurf.o video_processing2.o  
	$(CC) $^ -o $@ $(LDFLAGS)

mainTracking: main.o video_processing3.o 
	$(CC) $^ -o $@ $(LDFLAGS) 

.cpp.o:
	$(CC) $< -o $@ $(CFLAGS)

clean:
	rm -rf $(OBJ) $(OBJ_EXE) $(EXE)
