INCLUDE_DIRS =
LIB_DIRS = 
CC=g++

CDEFS=
CFLAGS= -O0 -g $(INCLUDE_DIRS) $(CDEFS)
LIBS= -lrt
CPPLIBS= -L/usr/lib -lopencv_core -lpigpio -lopencv_flann -lopencv_video -lpthread

HFILES= laneDetector.h
CFILES= 
CPPFILES=main.cpp LaneDetector.cpp

SRCS= $(HFILES) $(CFILES)
CPPOBJS= $(CPPFILES:.cpp=.o)

all:	main 

clean:
	-rm -f *.o *.d
	-rm -f main

distclean: clean
	-rm -f *.o *.d

main: $(CPPOBJS)
	$(CC) $(CFLAGS) -o $@ $(CPPOBJS) `pkg-config --libs opencv` $(CPPLIBS) $(LIBS)

depend:

%.o: %.cpp $(HFILES)
	$(CC) $(CFLAGS) -c $< -o $@
