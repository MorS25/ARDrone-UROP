CC=gcc
CFLAGS=  -c -Wall
LDFLAGS=$(shell pkg-config --libs opencv)
SRC=video_processing.c
SRC_EXE=main.c
EXE=main 
OBJ=$(SRC:.c=.o)
OBJ_EXE=$(SRC_EXE:.c=.o)

main: main.o video_processing.o $(OBJ)
	$(CC) $^ -o $@ $(LDFLAGS) 

.cpp.o:
	$(CC) $< -o $@ $(CFLAGS)

clean:
	rm -rf $(OBJ) $(OBJ_EXE) $(EXE)
