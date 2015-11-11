CXX=g++

all: car tests

car:
	$(CXX) kalman.cpp car_track.cpp -o car_track

tests:
	$(CXX) -I. -lgtest testmain.cpp tests.cpp -o testmain

prova:
	$(CXX) prova.cpp -o prova