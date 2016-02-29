CC=g++
ODEDIR=/home/amin/Desktop/ode-0.5
CURDIR=/home/amin/Creatures/Creatures
LFLAGS= -Wall -fno-rtti -fno-exceptions -g -I $(ODEDIR)/include -L$(CURDIR)  -L$(ODEDIR)/lib -L/usr/X11R6/lib -L/usr/X11/lib -L/usr/lib/X11R6 -L/usr/lib/X11 -lX11 -lGL -lGLU -lm -lode -ldrawstuff -lrandomc
CFLAGS= -c -fno-rtti -fno-exceptions -I $(ODEDIR)/include -fno-rtti -fno-exceptions -g
RCFLAGS = -c -g
PROJFILES = animat.o gene.o Neuron.o  definitions.o Ball.o limb.o
#RANDFILES = mersenne.cpp mother.cpp sfmt.cpp userintf.cpp stoc1.cpp stoc2.cpp stoc3.cpp wnchyppr.cpp fnchyppr.cpp
#RANDOFILES= $(RANDFILES:%.cpp=%.o)


all: evolve evolve1 evolve2 evolve3 evolve4 familytree # ssga readbest grab readgrab

#ssga: ssga.o $(PROJFILES)
#	$(CC)  $(PROJFILES) ssga.o $(LFLAGS) -o ssga
	
#readbest: readbest.o animat.o gene.o Neuron.o definitions.o Ball.o limb.o
#	$(CC) $(PROJFILES) readbest.o $(LFLAGS) -o readbest

#grab: grab.o animat.o gene.o Neuron.o definitions.o  Ball.o limb.o
#	$(CC) $(PROJFILES) grab.o $(LFLAGS) -o grab

#readgrab: readgrab.o animat.o gene.o Neuron.o definitions.o  Ball.o limb.o
#	$(CC) $(PROJFILES) readgrab.o $(LFLAGS) -o readgrab

familytree: familytree.o animat.o
	$(CC) $(PROJFILES) familytree.o $(LFLAGS) -o familytree

evolve: evolve.o animat.o definitions.o gene.o Neuron.o  Ball.o limb.o
	$(CC) $(PROJFILES) evolve.o $(LFLAGS) -o evolve

evolve1: evolve1.o animat.o definitions.o gene.o Neuron.o  Ball.o limb.o
	$(CC) $(PROJFILES) evolve1.o $(LFLAGS) -o evolve1

evolve2: evolve2.o animat.o definitions.o gene.o Neuron.o  Ball.o limb.o
	$(CC) $(PROJFILES) evolve2.o $(LFLAGS) -o evolve2

evolve3: evolve3.o animat.o definitions.o gene.o Neuron.o  Ball.o limb.o
	$(CC) $(PROJFILES) evolve3.o $(LFLAGS) -o evolve3

evolve4: evolve4.o animat.o definitions.o gene.o Neuron.o  Ball.o limb.o
	$(CC) $(PROJFILES) evolve4.o $(LFLAGS) -o evolve4

#grab.o: grab.cpp
#	$(CC) grab.cpp $(CFLAGS)

#readgrab.o: readgrab.cpp
#	$(CC) readgrab.cpp $(CFLAGS)

#readbest.o: readbest.cpp
#	$(CC) readbest.cpp $(CFLAGS)

#ssga.o: ssga.cpp
#	$(CC) ssga.cpp $(CFLAGS)

familytree.o: familytree.cpp
	$(CC) familytree.cpp $(CFLAGS)

evolve.o: evolve.cpp
	$(CC) evolve.cpp $(CFLAGS)

evolve1.o: evolve1.cpp
	$(CC) evolve1.cpp $(CFLAGS)

evolve2.o: evolve2.cpp
	$(CC) evolve2.cpp $(CFLAGS)

evolve3.o: evolve3.cpp
	$(CC) evolve3.cpp $(CFLAGS)

evolve4.o: evolve4.cpp
	$(CC) evolve4.cpp $(CFLAGS)



animat.o: animat.cpp animat.h
	$(CC) animat.cpp $(CFLAGS)

definitions.o: definitions.cpp definitions.h
	$(CC) definitions.cpp $(CFLAGS)

gene.o: gene.cpp gene.h
	$(CC) gene.cpp $(CFLAGS)

limb.o: limb.cpp limb.h
	$(CC) limb.cpp $(CFLAGS)

Neuron.o: Neuron.cpp Neuron.h
	$(CC) Neuron.cpp $(CFLAGS)

Ball.o: Ball.cpp Ball.h
	$(CC) Ball.cpp $(CFLAGS)

#librandomc.a: $(RANDOFILES)
#	rm -f $@
#	ar cqs $@ $(RANDOFILES)

#mersenne.o: mersenne.cpp
#	$(CC) $(RCFLAGS) mersenne.cpp

#mother.o: mother.cpp
#	$(CC) $(RCFLAGS) mother.cpp

#userintf.o: userintf.cpp
#	$(CC) $(RCFLAGS) userintf.cpp

#stoc1.o: stoc1.cpp
#	$(CC) $(RCFLAGS) stoc1.cpp

#stoc2.o: stoc2.cpp
#	$(CC) $(RCFLAGS) stoc2.cpp

#stoc3.o: stoc3.cpp
#	$(CC) $(RCFLAGS) stoc3.cpp

#wnchyppr.o: wnchyppr.cpp
#	$(CC) wnchyppr.cpp $(RCFLAGS)

#sfmt.o: sfmt.cpp
#	$(CC) sfmt.cpp $(RCFLAGS)

clean:
	rm -rf *o
#	rm -f librandomc.a