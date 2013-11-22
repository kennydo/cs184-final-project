CC = g++
ifeq ($(shell sw_vers 2>/dev/null | grep Mac | awk '{ print $$2}'),Mac)
	CFLAGS = -g -DGL_GLEXT_PROTOTYPES -I./include/ -I/usr/X11/include -DOSX
	LDFLAGS = -framework GLUT -framework OpenGL \
    	-L"/System/Library/Frameworks/OpenGL.framework/Libraries" \
    	-lGL -lGLU -lm -lstdc++
	EIGEN_INCLUDE = -I/usr/local/include/eigen3/
else
	CFLAGS = -g -DGL_GLEXT_PROTOTYPES
	LDFLAGS = -lglut -lGLU
	EIGEN_INCLUDE = -I/usr/include/eigen3/
endif
	
RM = /bin/rm -f 
all: kinematics 

kinematics: main.o joint.o 
	$(CC) $(CFLAGS) -o kinematics main.o joint.o $(LDFLAGS) $(EIGEN_INCLUDE)

main.o: main.cpp joint.h
	$(CC) $(CFLAGS) -c main.cpp $(EIGEN_INCLUDE)

joint.o: joint.cpp joint.h
	$(CC) $(CFLAGS) -c joint.cpp $(EIGEN_INCLUDE)

clean: 
	$(RM) *.o kinematics
 


