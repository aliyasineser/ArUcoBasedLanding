compile:
	g++ -std=c++11 arucoDetect.cpp `pkg-config --libs --cflags opencv` -laruco  -lglut -lGL -lSDL -lGLU -o markerDetect

run:
	./markerDetect

clean:
	rm -rf markerDetect arucoDetect.o

all:
	compile
	run
