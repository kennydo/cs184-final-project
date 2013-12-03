CC = g++
ifeq ($(shell sw_vers 2>/dev/null | grep Mac | awk '{ print $$2}'),Mac)
	CFLAGS = -g -DGL_GLEXT_PROTOTYPES -I./include/ -I/usr/X11/include -DOSX
	LDFLAGS = -framework GLUT -framework OpenGL \
    	-L"/System/Library/Frameworks/OpenGL.framework/Libraries" \
    	-lGL -lGLU -lm -lstdc++
	EIGEN_INCLUDE = -I/usr/local/include/eigen3/
else
	CFLAGS = -g -Wall -O2 -DGL_GLEXT_PROTOTYPES
	LDFLAGS = -lglut -lGLU -lGL
	EIGEN_INCLUDE = -I/usr/include/eigen3/
endif
	
RM = /bin/rm -f 
CPP_FILES = $(wildcard *.cpp)
OBJ_FILES = $(notdir $(CPP_FILES:.cpp=.o))

all: $(OBJ_FILES)
	$(CC) $(CFLAGS) -o kinematics $^ $(LDFLAGS) $(EIGEN_INCLUDE)

%.o: %.cpp
	$(CC) $(CFLAGS) -c -o $@ $< $(LDFLAGS) $(EIGEN_INCLUDE)

clean: 
	$(RM) *.o kinematics
