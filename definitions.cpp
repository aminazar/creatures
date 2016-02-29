#include "definitions.h"
#include <iostream>
#include <time.h>
#include "stocc.h"

StochasticLib1 sto((int)time(0));           // make instance of random library
int OUTPUTREDIRECT = TOFILE;
FILE *OUTPUTFILE;
int CORRIDOR = 0;
int WALLS = 0;
dGeomID lcorr, rcorr, walln, walls, wallw, walle;
int BOARD = 0;
int board [BSIDE][BSIDE];
dSpaceID globalspace;
extern dContact contact[MAXCONT];	// up to 4 contacts per box-box
dWorldID world;
dJointGroupID contactgroup;


//Amin: in animat.cpp
extern int WORLDTYPE;
extern int AIRCOLLISIONS;

void myprintf(const char *msg, ...)
{
    static va_list argp;
    static int nbcalls=0;
    if (OUTPUTREDIRECT == TONULL) return;
    if (OUTPUTREDIRECT == TOFILE)
    {
	if (nbcalls == 0)
	    OUTPUTFILE = fopen("./output", "w");
        if(!OUTPUTFILE){printf("\n\nWrite error!\n");exit(-1);}
	if (nbcalls == 50) fflush(OUTPUTFILE);
	if (nbcalls == 10000)
	{
	    nbcalls = 1;
	    fclose(OUTPUTFILE);
	    OUTPUTFILE = fopen("./output", "w");
	}
	va_start(argp, msg);
	vfprintf(OUTPUTFILE, msg, argp);
	va_end(argp);
    }
    else if (OUTPUTREDIRECT == TOSTDOUT)
    {
	va_start(argp, msg);
	vprintf(msg, argp);
	va_end(argp);
    }

}

void myexit(int i)
{
    myprintf("Exiting |\n");
    if (OUTPUTFILE) fflush(OUTPUTFILE);
    exit(i);
}


// exits program, write reason to "exitmessage.txt"
void mydie(const char *msg, ...)
{
    va_list argp;
    FILE *out;
    vprintf(msg, argp);
    fflush(stdout);
    out = fopen("exitmessage.txt", "w");
    va_start(argp, msg);
    vfprintf(out, msg, argp);
    va_end(argp);
    fflush(out);
    myexit(-1);
}


dReal gauss(dReal std)
{
    dReal x, y, expo;
    while (1)
    {
	x = (sto.IRandomX(0,1000)) / 100.0;
	if (sto.IRandomX(0,1)) x = -x;
	y = (1+sto.IRandomX(0,1000)) / 100.0;
	expo = (x / std); expo = expo*expo;
	if (y <  (exp (-expo/2.0) / (std * sqrt(2.0 * M_PI))))
	    return x;
    }
}

dReal perturbPositive(dReal x)
{
    dReal y;
    y = x + gauss(0.2);
    if (y < 0) y += 1;
    if (y > 1) y -= 1;
    return y;
}

dReal perturb(dReal x)
{
    dReal y;
    y = x + gauss(0.4);
    if (y < 1) y += 2;
    if (y > 1) y -= 2;
    return y;
}

dReal mytanh(dReal x)
{
    return tanh(3.0*x);
}

dReal sigmoid(dReal x)
{
    return 1.0 / (1.0+exp(-6.0*x));
}



int firstocc (int n, int *tab, int size)
{
    int i, found = -1;
    for (i=0; i < size; i++)
	if (tab[i] == n)
	{
	    found = i; break;
	}
    return found;
}

dReal invexp(dReal x)
{
    return exp (-2*x*x);
}

void anglesToPoint(dBodyID id, dReal l, dReal w, dReal h,
	dReal alpha, dReal beta, dVector3 result)
{

    // What is the intersection of a certain line, from the centre of the limb,
    // with horizontal angle alpha and vertical angle beta, and the surface of
    // the limb?

    dReal d;
    dReal x, y, z;

    l = l / 2.0; w = w / 2.0; h = h / 2.0;
    d = sqrt (h*h + w*w + l*l);
    z = d * sin(beta);
    d = d * cos(beta);  // => d = sqrt(x*x + y*y)
    x = d * cos(alpha);
    y = d * sin(alpha);
    if (x > l) x = l;
    if (y > w) y = w;
    if (z > h) z = h;
    if (x < -l) x = -l;
    if (y < -w) y = -w;
    if (z < -h) z = -h;
    dBodyGetRelPointPos (id, x, y, z, result);
}


// old utility functions, create features in the (flat!) environment

void drawCorridor()
{
    if (!CORRIDOR) return;
    static const dReal corr[3] = {100,1,50};
    dsSetColor(1,0,1);
    dsDrawBox (dGeomGetPosition(rcorr), dGeomGetRotation(rcorr), corr);
    dsDrawBox (dGeomGetPosition(lcorr), dGeomGetRotation(lcorr), corr);
}
void makeCorridor()
{
    if (!CORRIDOR) return;
    lcorr = dCreateBox (globalspace, 100, 1, 50);
    rcorr = dCreateBox (globalspace, 100, 1, 50);
    dGeomSetPosition (rcorr, 0, ((float)CORRSPACE / 2.0) + 0.6, 1);
    dGeomSetPosition (lcorr, 0, -((float)CORRSPACE / 2.0) - 0.6, 1);
}
void makeWalls()
{
    if (WALLS==0) return;
    walln = dCreateBox (globalspace, 1, WALLSIZE, 30);
    walls = dCreateBox (globalspace, 1, WALLSIZE, 30);
    wallw = dCreateBox (globalspace, WALLSIZE, 1, 30);
    walle = dCreateBox (globalspace, WALLSIZE, 1, 30);
    dGeomSetPosition (walln, -(WALLSIZE/2.0), 0, 15);
    dGeomSetPosition (walls, (WALLSIZE/2.0), 0, 15);
    dGeomSetPosition (wallw, 0, (WALLSIZE/2.0), 15);
    dGeomSetPosition (walle, 0, -(WALLSIZE/2.0), 15);
}
void drawWalls()
{
    if (!WALLS) return;
    const dReal ns[3] = {1,WALLSIZE,30};
    const dReal ew[3] = {WALLSIZE,1,30};
    dsSetColor(1,0,1);
    dsDrawBox (dGeomGetPosition(walln), dGeomGetRotation(walln), ns);
    dsDrawBox (dGeomGetPosition(walls), dGeomGetRotation(walls), ns);
    dsDrawBox (dGeomGetPosition(walle), dGeomGetRotation(walln), ew);
    dsDrawBox (dGeomGetPosition(wallw), dGeomGetRotation(walls), ew);
}

void resetBoard()
{
    int i, j;
    for (i=0; i < BSIDE; i++) for (j=0; j < BSIDE; j++) board[i][j] = 1;
}
void makeBoard()
{
    if (BOARD==0) return;
    walln = dCreateBox (globalspace, 1, BSIDE*SQSIZE, 30);
    walls = dCreateBox (globalspace, 1, BSIDE*SQSIZE, 30);
    wallw = dCreateBox (globalspace, BSIDE*SQSIZE, 1, 30);
    walle = dCreateBox (globalspace, BSIDE*SQSIZE, 1, 30);
    dGeomSetPosition (walln, 0, BSIDE*SQSIZE / 2, 15);
    dGeomSetPosition (walls, BSIDE*SQSIZE, BSIDE*SQSIZE/2, 15);
    dGeomSetPosition (wallw, BSIDE*SQSIZE/2, 0, 15);
    dGeomSetPosition (walle, BSIDE*SQSIZE/2, BSIDE*SQSIZE, 15);
    resetBoard();
}
void drawBoard()
{
    if (!BOARD) return;
    int i, j;
    const dReal ns[3] = {1,BSIDE*SQSIZE,30};
    const dReal ew[3] = {BSIDE*SQSIZE,1,30};
    const dReal sqsides[3] = {SQSIZE-0.2,SQSIZE-0.2,0.05};
    dMatrix3 IdMat = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
    dReal tab1[3] = {0, 0, 0.05};
    dsSetColor(1,0,1);
    dsDrawBox (dGeomGetPosition(walln), dGeomGetRotation(walln), ns);
    dsDrawBox (dGeomGetPosition(walls), dGeomGetRotation(walls), ns);
    dsDrawBox (dGeomGetPosition(walle), dGeomGetRotation(walln), ew);
    dsDrawBox (dGeomGetPosition(wallw), dGeomGetRotation(walls), ew);
    for (i=0; i < BSIDE; i++)
	for (j=0; j < BSIDE; j++)
	{
	    if (board[i][j]) dsSetColor (1, 1, 0);
	    else dsSetColor (0, 1, 1);
	    tab1[0] = i * SQSIZE + SQSIZE / 2;
	    tab1[1] = j * SQSIZE + SQSIZE / 2;
	    dsDrawBox (tab1, IdMat, sqsides);
	}
}


// start simulation - set viewpoint
void start()
{
    float xyz[3] = {6,0,30};
    float hpr[3] = {-179.9,-57,0};
    if (WORLDTYPE == FLATWORLD)
    {
	xyz[0] = 1.7; xyz[1] = -7; xyz[2] = 6.8;
	hpr[0] = 104; hpr[1] = -44; hpr[2] = 0.0;
    }
    dsSetViewpoint (xyz,hpr);
    printf ("Simulation started !");
}


// called when a key pressed

void command (int cmd)
{
  if (cmd == 'e' || cmd == 'E') {
      myprintf("You pressed 'E'\n");
  }
}


void airCallback (void *data, dGeomID o1, dGeomID o2)
{
    // utility function,
    // used for detection of collisions in mid-air when dropped
  if (dGeomIsSpace (o1) || dGeomIsSpace (o2))
  {
      mydie("Damn, one of two colliders is a space !\n");
  }
  if (dCollide(o1,o2,MAXCONT,&contact[0].geom,sizeof(dContact)))
  {
      AIRCOLLISIONS = 1;
      return;
  }
}


    void drawSlant()
    {
	dVector3 sides = {.1,20,20}; // vertical 'slice'
	dVector3 centre = {0, 0, 0};
	dMatrix3 rot;
	dRFromAxisAndAngle(rot, 0, 1, 0, -(M_PI / 4.0));
    dsSetColor(1,0,1);
	dsDrawBox(centre, rot, sides);
    }


void destroyWorld()
{
    dWorldDestroy (world);
    dJointGroupDestroy (contactgroup);
    dSpaceDestroy (globalspace);
}

void print_3d_vect(dReal *vect,const char *name)
{
    std::cout<<name<<"("<<vect[0]<<","<<vect[1]<<","<<vect[2]<<")\n";
}

void name_file(char *name,int num,int generation)
{
    sprintf(name,"g%03d-%04d",generation,num);
    strcat(name,".dat");
}