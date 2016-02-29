// (c) The university of Birmingham and quite possibly the University of
// Portsmouth, 2004-2007.
// Academic and educational uses only.
#ifndef _ANIMAT_H
#define	_ANIMAT_H
#include <time.h>
#include <algorithm>
#include <stack>
#include <vector>
#include <deque>
#include "definitions.h"
#include "Neuron.h"
#include "gene.h"
#include "limb.h"
#include "Ball.h"

// Refuses conns from actuators !
// But allows them from self !
/*
template<class T> class safeVar
{
    T var;
    bool is_constructed;
public:
    bool isSafe(){return is_constructed;}
    safeVar(){is_constructed=false;}
    safeVar(T value){var = value;is_constructed=true;}
    void setVar(T value){var = value;is_constructed=true;}
    T &getVar(){if(!is_constructed){myprintf("\nABORT:UNSAFE ACCESS!\n");exit(-1);} return var;}
};*/

using namespace std;

class vect2d
{
    dReal x,y;
public:
    vect2d(){x=0.0;y=0.0;}
    vect2d(const vect2d& c){x=c.x;y=c.y;}

    vect2d(dReal X,dReal Y){x=X;y=Y;}

    void operator+=(const vect2d &sec)
    {x+=sec.x;y+=sec.y;}

    vect2d operator+(const vect2d &sec)const
    {vect2d tmp(x+sec.x,y+sec.y);return tmp;}

    dReal size() const
    {return sqrt(x*x + y*y);}

    dReal operator&(const vect2d &sec)const
    //dot product
    {return x*sec.x+y*sec.y;}

    void operator*=(dReal scalar)
    {x*=scalar;y*=scalar;}

    vect2d operator*(dReal scalar)const
    {vect2d tmp(x*scalar,y*scalar);return tmp;}

    vect2d o_projection_on(const vect2d &sec)const
    //orthogonal projection
    {vect2d tmp(sec.x,sec.y);tmp*=(*this&sec)/(sec&sec);return tmp;}

    dReal projection_on(const vect2d &sec)const
    //scalar projection
    {if(sec.size()==0) return 0.0;
     return (*this&sec)/sec.size();}

    void print(char *title=0)
    {if(title)printf(title);printf("(%f,%f)\n",x,y);}
};

class Animat {
public:
    char parent[2][20];
    char name[20];
    int alive;
    int nbLimbs;
    int liked;
    int symYinputs;
    int idxgen;
    int ID, PID1, PID2; // MUST BE FILLED BY THE PROGRAM
    Animat *closestanimat;
    dReal oldX,oldY,avg_score,first_height;
    vect2d motionSum;
    stack<vect2d> Motion;
    stack<dReal> Height;
    vector<dReal> scoreVector;

    //dJointGroupID joints;
    dSpaceID space;

// NOTE: both the genome and the repres must be ORDERED and COMPACT.
// Ordered = any gene can only occur in the array BEFORE the genes that
// describe its children limbs
// Compact = a "dead" (i.e. non-existent) gene can NEVER be followed by a
// non-dead gene.
// heritable genetic data
    gene genome[MAXGENES];

// actual representation of the creature, post-development
    gene repres[MAXLIMBS]; 

// physica data for each limb, including ODE data structures
    Limb limbs[MAXLIMBS];
    Limb *head;

    Limb *createLimb (int index, dReal  l, dReal w, dReal h, dReal x, dReal y,
			dReal z, dReal alpha);
    Limb *addLimb (Limb *tr, int idxGen);
//    dReal x, y, z;
    Animat();

    bool read(int generation,int num);

    bool read(const char *s,bool new_mode=true);
    
    void save(char *s);

    void readOld(const char *s);
    
    void saveOld(const char *s);
   
    int nblimbs();
    
    void checkGenome();
    
    int nbGenes();
   
    void displayGenome();
  
    void displayGenomeFull();

    void displayRepres();

    void compress();

    dReal curDamageFrom(Animat *other);
    
    void addNewGeneOld();

    void addNewGene();

    void removeGene(int a); // DOES NOT COMPRESS THE GENOME !
   
    void remove1Gene(int a);

    void insertFromOld(Animat *B, int nfrom, int nto);

    void insertFromLessOld(Animat *B, int dadfrom, int dadto);

    void checkConnections();
      
    void setImmunityTimer(int time);
  
    void resetDamages();
  
    dReal getTotalDamageDone();
   
    dReal getTotalDamage();
    
    void deleteUnconnNeurons();
    
    void copyFrom(Animat *B);

    void copyGenomeOnlyFrom (Animat *B);
      
    void crossWith (Animat *B);
 
    void reassignBadConns();
       
    int insertFrom(Animat *B, int from, int dadto);
  	    	
    void pickFrom(Animat *B);
   
    bool graftWith(Animat *B);
    
    void clearNeur(int limb, int num);
  
    void delNeur(int limb, int num);
 		
    void randConn(int limb, int neur, int conn);

    bool mutate();

    void findClosestAnimat();

    void fillSensors();
    
    void actuate();

    void makeNeuronsLimb(int limb);
    
    void randConnectionsLimb(int limb);
 
    void randConnectionsNeur(int limb, int neur);
  
    void printRepres();
  
    void printGenome();
   
    void printGenomeFull();

    void readRepres(dReal x, dReal y, dReal z, dReal alpha);

    void randGenome();
  
    // Note: all "push.." functions must be called while the animat is alive
    // (i.e. after it's been generated) - obviously.
    
    void pushBeforeXVert(dReal X);
      
    void pushBehindXVert(dReal X);
    
    void pushBehindX(dReal X);
    
    void pushBeforeX(dReal X);

    int isInSublimbGenome(int limb, int anc);
   
    int isInSublimbRepres(int limb, int anc);
   
    void transGenome();
   
    void transGeneSeg(int zegene, int zedad, int numrec, int symm);
   
    void transNeurons();
   
    void dev1RecInGenome();
   
    void transGeneDev1Rec(int zegene, int zedad, int numrec, int symm, int genetodev);
     
    void generate(dReal nx, dReal ny, dReal alpha);

    void remove();
    
    void setPos(dReal nx, dReal ny);
   

    void setPos(dReal nx, dReal ny, dReal nz);
 
    void rotateByQ(dQuaternion q);
  
    void dropRandomOnSphere();
     
    dReal getLongi();
   
    dReal getLati();
      
    void dropLatiLongi(dReal lati, dReal longi);
     
    int test_for_intra_coll();

    dReal height();
    dReal damage();
    dReal newScore(dReal damage_coef);
    dReal score();
    void update(dReal stepLength,dReal minHeight=0.0,dReal z_stretch=1);//, dReal progress);
    void reset();
    dReal averageScore(bool current=false);
    //if 'current' is true and current score is available returns that, otherwise
    //calculates it.
};



void addDamage(Animat *A, Animat *B, dReal damage);

void doWorld (int pause, dReal step, int fast);

void resetScene();

void initWorld();

void simLoop (int pause);

class genome_saver
{
public:
    gene genome[MAXGENES];
    dReal score;
    int liked;
    char parent[2][20];
    genome_saver(gene *g,dReal Score,const char *parent1,const char *parent2,int Liked=0)
    {
        for(int i=0;i<MAXGENES;i++)
            genome[i]=g[i];
        score = Score;
        liked = Liked;
        strcpy(parent[0],parent1);
        strcpy(parent[1],parent2);
    }

    genome_saver(){}

    void copy_on(Animat *anim)
    {
        for(int i=0;i<MAXGENES;i++)
            anim->genome[i]=genome[i];
        anim->avg_score=score;
        anim->liked=liked;
        strcpy(anim->parent[0],parent[0]);
        strcpy(anim->parent[1],parent[1]);
    }
};

#endif	/* _ANIMAT_H */