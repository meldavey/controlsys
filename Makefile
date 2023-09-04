
CC=g++
CXXFLAGS = -std=c++11
CFLAGS=-I.
DEPS = ControlSystem.h
OBJ = ControlSystem.o ControlSystemTest.o 

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

ControlSystemTest: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

