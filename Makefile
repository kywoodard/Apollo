 CXXFLAGS += -I/usr/local/include -std=c++11
 LDFLAGS += -L/usr/local/lib

all: rocket_ode.cpp
	g++ $(CXXFLAGS) -o rocketSim rocket_ode.cpp