g++ -std=c++11 arucoDetect.cpp `pkg-config --libs --cflags opencv` -o markerDetect
./markerDetect
