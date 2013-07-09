CC=gcc
CFLAGS=  -c -Wall
LDFLAGS=$(shell pkg-config --libs opencv)
SRC=video_processing3.c
SRC_EXE=main.c
SRC_EXE=main.c
EXE=main 
OBJ=$(SRC:.c=.o)
OBJ_EXE=$(SRC_EXE:.c=.o)

main: main.o $(OBJ)
	$(CC) $^ -o $@ $(LDFLAGS) 

.cpp.o:
	$(CC) $< -o $@ $(CFLAGS)

clean:
	rm -rf $(OBJ) $(OBJ_EXE) $(EXE)
