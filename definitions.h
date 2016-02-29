/* 
 * File:   definitions.h
 * Author: amin
 *
 * Created on 03 April 2010, 01:00
 */

#ifndef _DEFINITIONS_H
#define	_DEFINITIONS_H
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <stdlib.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <time.h>
#include "Ball.h"
#include "limb.h"

// select correct drawing functions
#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox dsDrawBoxD
#endif


// Note: Many of the following constants are actually not used in the currents
// version of the code.

// ODE constants, obtained through painful parameter tuning
#define MYCFM 0.01
#define MYERP 0.015


// MAX NUMBER OF ANIMATS ALIVE IN THE SIMULATION AT ANY TIME
// Increasing this much beyond need may reduce performance
// Putting more animats in the simulation will break the system
#define REGISTSIZE  100


#define PROBAMUT  2//4 // probability of mutation - TRY TO INCREASE THIS !
#define WORLDRADIUS 15 // for spherical world
#define DROPHEIGHT 10 // for spherical world
#define HURTIMMUNITYTIMER 200
#define VICTORY 123
#define DEFEAT 321
#define NONE 444
#define SPEEDMULT 6.0
#define MAXTHRES 0.5
#define MAXRECUR 3
#define NBCYCLES 10000000
#define INITIALDIST 1.5
#define MYRANDSEED 0 //  if 0, then random seed is taken from /dev/urandom
#define SIDE (1.0f)	// side length of a box
#define MAXFORCE 100 // 3 ?
#define NBNEURCYCLES 6  // 2 ?
#define STEP 0.001
#define PROBAINITRECUR 50 // probability of recursion in initial random genome
#define PROBAINITREF 95 // same for reflection
#define MINGENES 4 // minimum number of genes allowed in a genome
#define MAXGENES 5 // maximum
#define MAXLIMBS 18 // maximum number of limbs allowed after development
#define CORRSPACE 12
#define WALLSIZE 150
#define MAXCONFROM 10
#define MAXCONT 4
#define BSIDE 24
#define SQSIZE 8
#define PROBANEUREXISTS 66 // 50
#define PROBANEURSENS 60 // 50, with 2 sensor types
#define PROBACONNEXISTS 90
#define MAXNEUR 10
#define SIGMOID 10
#define INVEXP 20
#define TANH 30
#define INTER 0
#define ACTUATOR 7
#define SENSPROP 8
#define TOFILE 54
#define TONULL 55
#define TOSTDOUT 56
#define REFBOTH 44
#define REFORIG 45
#define REFSYMM 46
#define RECSELF 90
#define RECDAD 91
#define RECSON 92
#define SPHERICWORLD 505
#define FLATWORLD 506

// forces limb 0 to possess a SENSCLOSESTANIMX and SENSCLOSESTANIMY sensor.
#define ENFORCESENSORSINTRUNK 1



// SENSORTYPES defines which sensors are to be used in the simulation (besides
// proprioceptors, which are always used).  MODIFY THIS (and also
// NBUSEDSENSORTYPES) if you want to use more or fewer sensors in your
// simulation - of course, don't forget to update fillSensors as well.

#define NBSENSORTYPES 6
// UNCOMMENT THIS (and comment out the previous passage) IF YOU WON'T USE THE
// BALL/BOX IN YOUR SIMULATIONS
//
//#define NBSENSORTYPES 3 int SENSORTYPES[NBSENSORTYPES] = {SENSCLOSESTANIMX,
//SENSCLOSESTANIMY, SENSTOUCH};
void myprintf(const char *, ...);

void myexit(int);
// exits program, write reason to "exitmessage.txt"
void mydie(const char *, ...);

dReal gauss(dReal);

dReal perturbPositive(dReal );

dReal perturb(dReal );

dReal mytanh(dReal );

dReal sigmoid(dReal );

int firstocc (int , int *tab, int size);

dReal invexp(dReal );

void airCallback(void *data, dGeomID o1, dGeomID o2);

void nearCallback (void *data, dGeomID o1, dGeomID o2);

class Animat;

void anglesToPoint(dBodyID id, dReal l, dReal w, dReal h,
	dReal alpha, dReal beta, dVector3 result);


// old utility functions, create features in the (flat!) environment

void drawCorridor();

void makeCorridor();
void makeWalls();
void drawWalls();
void resetBoard();
void makeBoard();
void drawBoard();

// start simulation - set viewpoint
void start();

// called when a key pressed
void command (int cmd);

void airCallback (void *data, dGeomID o1, dGeomID o2);

void drawSlant();

void destroyWorld();

void initWorld();

void print_3d_vect(dReal *vect,const char *name);
void name_file(char *name,int num,int generation);

#endif	/* _DEFINITIONS_H */

