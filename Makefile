# note: look at specific module, fn, app makefiles for more guidence but
# this is the basic template
#
# Feature Based Mono Slam:  gmake fbmslam
#
# sample library build command:
#
# gcc -I/path/to/opensift/include/ -L/path/to/opensift/lib/ yourcode.c \
# -o yourexecutable -lopensift
#
# -----------------------------------------------------------------------------
# - Define macros
# ---------------
SHELL = /bin/csh

#include debug_info

#opencvPath=/lu1/smagri/uni/subj/ar/project/

#opencvINC = ${openshiftPath}/include/
opencvINC =
#opencvLIB = /usr/lib/x86_64-linux-gnu/libopencv
opencvLIB = `pkg-config --libs opencv`
#
# ParallaxBA library /usr/local/lib/libParallexBAImp.a
pbaLIB = -lParallaxBAImp
#
# Other libs:
eigen3LIB = -leigen3
suitesparseLIB = -lsuitesparse

LibsAndPaths = ${opencvLIB} ${pbaLIB} -leigen3 -lsuitesparse

#OTHER_ARG_aprog = dscomm.o

#CC=gcc
#
CC = g++

#should work
#CFLAGS = `pkg-config --cflags opencv`
#
CFLAGS = `pkg-config --cflags opencv eigen3` -I/usr/local/include/ParallaxBA \
	-I/usr/include/suitesparse

#opencvCFLAGS = `pkg-config --cflags opencv`
#
#CFLAGS = -Iinclude -O -Wall -pedantic -g
# !debugging:
#CFLAGS = -Iinclude -O -Wall -ansi -pedantic

# -----------------------------------------------------------------------------
# - Targets, prerequisites and their build rules
# ----------------------------------------------
#aprog: aprog.c aprog_dependancy1.o ... aprog_dependancyN.o 
#        ${CC} ${CFLAGS} ${APROG}.c ${OTHER_ARG_aprog} -o ${APROG}
#
fbmslam: fbmslam.cpp
	${CC} ${CFLAGS} fbmslam.cpp -o fbmslam \
	${opencvLIB}

# noworks:
#
#hello-world: hello-world.cpp
#	${CC} ${CFLAGS} ${opencvLIB} hello-world.cpp -o hello-world
#
hello-world: hello-world.cpp
	${CC} ${CFLAGS} hello-world.cpp -o hello-world ${opencvLIB}


#include debug_targets

# -----------------------------------------------------------------------------
# - misc commands
# ---------------
clean:
	\rm fbmslam

cleanhw:
	\rm hello-world
