#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "definitions.h"
#include "animat.h"
#include <iostream>
#include <algorithm>

#define ABS(x) (x<0?-x:x)
#define POPULATION_SIZE 5000
#define NEXPRMNTS 10
#define CHOSEN_FRACTION 10

extern long int tot_time;
long int time_limit = 10000;
dReal invCosh10 = 2.993222846;
extern dsFunctions fn;
extern int WORLDTYPE;
extern int OUTPUTREDIRECT;
extern int BALL;
extern int VISUAL;
extern dMatrix3 IDENTITY;
extern int WALLS;
extern int CORRIDOR;
extern dWorldID world;
extern dJointGroupID contactgroup;
extern dSpaceID globalspace;
extern int NOACTUATE;
//extern dReal DAMAGETABLE[REGISTSIZE][REGISTSIZE];

Animat *curAnim;
int anim_ind=0,exp_ind=0;
int failure=0;

class animIndex
{
        int i;
    public:
        animIndex(){}
        animIndex(int x){i=x;}
        void setIndex(int x){i=x;}
        //animIndex &operator=(&animIndex &ind){i=ind.i;return *this;}
        int getIndex() const
        {return i;}
};

bool operator<(const animIndex&,const animIndex&);

class animCapsule
//A class with singleton pattern
{
    animIndex anim_index[POPULATION_SIZE];
    Animat anim[POPULATION_SIZE];
    static bool is_constructed;
    static animCapsule *instance;
    animCapsule()
    {
        animCapsule::is_constructed=false;
        for(int i=0;i<POPULATION_SIZE;i++)
            anim_index[i].setIndex(i);
    }
public:
    static animCapsule *getInstance()
    {
        if(is_constructed)
            return instance;
        instance = new animCapsule();
        is_constructed=true;
        return instance;
    }
    
    Animat &at(int i)
    {
        return anim[anim_index[i].getIndex()];
    }
    
     void sort()
    {
        std::sort(anim_index, anim_index+POPULATION_SIZE);
        cout<<"\nSorted:\n";
        for(int i=0;i<POPULATION_SIZE;i++)
            cout<<i<<"->"<<anim_index[i].getIndex()<<endl;
    }

    friend  bool operator<(const animIndex&,const animIndex&);
};

bool animCapsule::is_constructed;
animCapsule *animCapsule::instance;
bool loaded = false;

inline bool operator<(const animIndex& ind1,const animIndex& ind2)
{return animCapsule::getInstance()->at(ind1.getIndex()).averageScore()>animCapsule::getInstance()->at(ind2.getIndex()).averageScore();}

void name_file(char *name,int num,char *pre=0);

void simLooop(int );
void dooWorld (int , dReal step, int fast);
int isCoolliding(Animat *elim);
void newAnim(Animat &);
void replace(Animat *elim, Animat *A, Animat *B);
void after_timer();
void key_command(int cmd);

Animat ani;

void (*timer_ends) ()   =   0;
void (*experiment_succeed)()    =   0;
void (*experiment_failed)() =   0;

int main (int argc, char **argv)
{
    BALL = 0;
    for(int i=0;i<=NEXPRMNTS;i++)
        failure[i]=0;
    WORLDTYPE = FLATWORLD;
    OUTPUTREDIRECT = NULL;
    initWorld();

    fn.step = &simLooop;
    timer_ends = &after_timer;

    if(argc>1)
    {
        for(int a=1;a<argc;a++)
        {
            if(strcmp(argv[a],"load")==0)
            {
                int max=(atoi(argv[a+1]))?atoi(argv[a+1]):POPULATION_SIZE/CHOSEN_FRACTION;
                do_generate = false;
                for(int i=0;i<max;i++)
                {
                    char fname[20];name_file(fname,i,"000g-");
                    curAnim = &animCapsule::getInstance()->at(i);
                    curAnim->read(fname);
                    std::cout<<fname<<" is loaded.\n";
                    loaded=true;
                }
                break;
                //exit(0);
            }
            else if(strcmp(argv[a],"show")==0)
            {
                anim_ind=POPULATION_SIZE - 1;
                curAnim = &animCapsule::getInstance()->at(anim_ind);
                curAnim->read(argv[a+1]);
                    std::cout<<argv[a+1]<<" is loaded.\n";
                loaded=true;
                break;
            }
            else if(strcmp(argv[a],"evotest")==0)
            {
                for(int i=0;i<19;i++)
                {
                    char fname[20];name_file(fname,i,"000g-");
                    curAnim = &animCapsule::getInstance()->at(i);
                    curAnim->read(fname);
                    std::cout<<fname<<" is loaded.\n";
                    loaded=true;
                }
            }
            else if(strcmp(argv[a],"evo")==0)
            {
                int min=(atoi(argv[a+1]))?atoi(argv[a+1]):POPULATION_SIZE/CHOSEN_FRACTION;
                int max=(atoi(argv[a+2]))?atoi(argv[a+2]):POPULATION_SIZE;
                do_generate = true;
                char fname[20];
                for(;anim_ind<min;anim_ind++)
                {
                    curAnim = & animCapsule::getInstance()->at(anim_ind);
                    curAnim->read(fname);
                    std::cout<<fname<<" is loaded.\n";
                }
                for(;anim_ind<max;anim_ind++)
                {
                    curAnim = & animCapsule::getInstance()->at(anim_ind);
                    int ind1=sto.IRandomX(0,min-1),
                        ind2=sto.IRandomX(0,min-2);
                    if(ind2==ind1)ind2++;
                    replace(curAnim,& animCapsule::getInstance()->at(ind1),& animCapsule::getInstance()->at(2));
                }
                
            }
        }
    }
    cout<<anim_ind;
    newAnim(animCapsule::getInstance()->at(anim_ind));
    VISUAL=1;
    dsSimulationLoop (argc,argv,352,288,&fn);
    destroyWorld();
}


void newAnim(Animat &anim)
{
    curAnim=&anim;
    efforts++;
    bool is_colliding;
    if(do_generate && !loaded)//do
    {
        anim.randGenome();
        anim.checkGenome();
        if(is_colliding = isCoolliding(&anim))
        {failure[0]++;cout<<failure[0]<<endl;if(!sto.IRandom(0,9))is_colliding=false;}
    }//while(is_colliding);


    anim.generate(0,0,0);
    //std::cout<<"Animat Genome\n";
    //anim.displayGenomeFull();
    anim.displayRepres();
    anim.pushBehindXVert(0);
    printf("OK\n");
    tot_time = 0;
}

void simLooop(int pause)
{
// this is the loop function to be called when performing graphical
    // simulations (see readbest.cpp and visual.cpp for examples)
    //
    // It draws everything and calls doWorld()
    int i, j;
    i = 0;
    static dReal p[3] = {0, 0, 0};

  for (j=0; j < curAnim->nblimbs(); j++)
  {
      float T = 1.0/(curAnim->limbs[j].damage+1);
      switch(curAnim->limbs[j].type)
      {
          case Limb::HEAD:
              dsSetColor(T, 0, T);
              break;
          case Limb::TORSO:
              dsSetColor(T, T, T);
              break;
          case Limb::LIMB:
                dsSetColor(T/2, T, 0);
      }
      dsDrawBox (dBodyGetPosition(curAnim->limbs[j].id),
      dBodyGetRotation(curAnim->limbs[j].id),
      curAnim->limbs[j].sides);
  }
      
  if (WALLS) drawWalls();
  if (CORRIDOR) drawCorridor();

  if(tot_time>time_limit)
      timer_ends();
  

  dooWorld(pause, STEP,false);
}

void dooWorld (int pause, dReal step, int fast)
{

    // perform a cycle of the simulation.


    // initialisations...
    int noreg=1;
    if(!pause)tot_time++;
    
    // collision detection
    dSpaceCollide (globalspace,0,&nearCallback);

    if (!pause)
    {
		noreg = 0;

		             
                for (int l=0; l < curAnim->nblimbs(); l++)
		{                    
 
		    // update some tracing data
		    for (int n =0; n < 12; n++)
			curAnim->limbs[l].oldrot[n] =
			    dBodyGetRotation(curAnim->limbs[l].id)[n];

		    for (int n=0; n < 3; n++)
			curAnim->limbs[l].oldpos[n] =
			    dBodyGetPosition(curAnim->limbs[l].id)[n];
		}

		// perform a neural update cycle
		if (curAnim->alive)
                {
                    curAnim->actuate();
                    curAnim->update(step);//,((dReal)tot_time/time_limit)*invCosh10);
                }

               	else
		{
		    mydie(
		    "Damn ! Animat registered but not alive in doWorld !\n");
		}
		/*if (VISUAL)
		    myprintf("Output neuron 3 limb 0: %f\n",
			    anim.repres[0].neurons[3].out);*/
	   

	if (fast)
	    dWorldQuickStep(world, step); // quick, default
	else
	    dWorldStep (world,step); // slower, more precise
	tot_time++;
    }
    dJointGroupEmpty (contactgroup);
}

int isColliding(Animat *elim)
{
    NOACTUATE = 1;
    elim->generate(-50,0,0);
    if (elim->nblimbs() >= MAXLIMBS)
    {
	myprintf("Too big !\n");
	elim->remove();
	return 1;
    }
    if (elim->test_for_intra_coll())
    {
	myprintf("Self-Collided - 1st pass!\n");
	elim->remove();
	return 1;
    }
    if(elim->limbs[0].id)dooWorld(0, STEP, true);
    if (elim->test_for_intra_coll())
    {
	myprintf("Self-Collided - 2nd pass!\n");
	elim->remove();
	return 1;
    }
    elim->remove();
    NOACTUATE=0;
    return 0;
}

void name_file(char *name,int num,char *pre)
{
    if(pre)
        sprintf(name,"%s%04d",pre,num);
    else
        sprintf(name,"%04d",num);
    strcat(name,".dat");
}

// replaces elim's genome with either a recombination between A and B, or a
// mutated version of A or B.
void replace(Animat *elim, Animat *A, Animat *B)
{
    static int nb = 0;
    int wrong = 0;

    nb++;
    A->checkGenome();
    B->checkGenome();
    do{
	myprintf("* %d replacement\n", nb);

	switch (sto.IRandomX(0,5))
	{
	    case 0: myprintf("Cross - \n");
		    if (sto.Bernoulli(0.5))
		    {
			elim->copyFrom(A);
			elim->crossWith(B);
		    }
		    else
		    {
			elim->copyFrom(B);
			elim->crossWith(A);
		    }
		    break;
	    case 1: myprintf("Graft - \n");
		    if (sto.Bernoulli(0.5))
		    {
			elim->copyFrom(A);
			elim->graftWith(B);
		    }
		    else
		    {
			elim->copyFrom(B);
			elim->graftWith(A);
		    }
		    break;
	    case 2: myprintf("Pick - \n");
		    if (sto.Bernoulli(0.5))
		    {
			elim->copyFrom(A);
			elim->pickFrom(B);
		    }
		    else
		    {
			elim->copyFrom(B);
			elim->pickFrom(A);
		    }
		    break;
	    case 3: myprintf("Mutate A - \n");
		    elim->copyFrom(A);
		    elim->mutate();
		    elim->mutate();
		    break;
	    case 4: myprintf("Mutate B - \n");
		    elim->copyFrom(B);
		    elim->mutate();
		    elim->mutate();
		    break;
	}

	//   else
	//	crossoverOld(0, select(), select());

	elim->mutate();
	elim->reassignBadConns();

	elim->checkGenome();
	elim->checkConnections();

	wrong = isCoolliding(elim);

    } while(wrong);
}

void after_timer()
{
      cout<<"anim "<<anim_ind<<" expriment "<<exp_ind++<<") score:"<<curAnim->newScore()<<"\n";
      curAnim->motionSum.print("motionSum");
      std::cout<<"damage: "<<curAnim->damage()<<"\n";
      curAnim->remove();

      if(curAnim->score()<2&&!loaded)
      {
          curAnim->reset();
          failure[exp_ind+1]++;
          do_generate=true;
          exp_ind=0;
          newAnim(*curAnim);
      }
      else if(exp_ind<NEXPRMNTS)
      {
          do_generate=false;
          newAnim(*curAnim);
      }
      else
      {
          cout<<"average score:"<<curAnim->averageScore()<<endl;
          char fname[20];
          name_file(fname,anim_ind,"000g-");
          curAnim->save(fname);
          do_generate=true;
          anim_ind++;
          if(anim_ind<POPULATION_SIZE)
          {
                newAnim(ani);//animCapsule::getInstance()->at(anim_ind));
                exp_ind=0;
          }
          else
          {
              /*
              char fname[8];
              animCapsule::getInstance()->sort();
              for(int i=0;i<POPULATION_SIZE/CHOSEN;i++)
              {
                  name_file(fname,i);
                  animCapsule::getInstance()->at(i).save(fname);
                  std::cout<<"\n"<<fname<<" is saved.";
              }*/
              cout<<"\nEfforts: "<<efforts<<endl;

              for(int i=0;i<=NEXPRMNTS;i++)
                  cout<<"Failure at experiment "<<i<<": "<<failure[i]<<endl;

              exit(0);
       }
      }
}

void key_command(int cmd)
{
    if(curAnim)
    {
        if(cmd=='s'||cmd=='S')
        {
            char fname[20];
              name_file(fname,anim_ind,"000gs-");
              curAnim->save(fname);
        }
        if(cmd=='l'||cmd=='L')
        {
            curAnim->liked++;
        }
    }
}