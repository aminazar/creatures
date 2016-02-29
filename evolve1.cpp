#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "definitions.h"
#include "animat.h"
#include <iostream>
#include <algorithm>
#include <fstream>
#include <time.h>
#include <dirent.h>
#include <vector>
#include <string>
#include <sstream>
#include <unistd.h>

#define POPULATION_SIZE 94
#define NEXPRMNTS 1
#define ELITE_SIZE 50
#define CREATURES_DIR "."
#define TAB char(9)
#define COMMA ','
#define PARALLELISM_SIZE 4
#define INSTANCE_NO 0
#define SQR(x) (x)*(x)
#define MIN_SCORE 10
typedef pair<dReal,int> iScore;

//inline bool operator<(const iScore &f, const iScore &s)
//{return f.first>s.first;}

class averager:public vector<iScore>
{
 public:

    dReal avg()
    {
        if(empty())
            return 0.0;
        dReal ret=0.0;
        for(iterator i=begin();i<end();i++)
            ret+= i->first;
        return ret/size();
    }

    dReal exp_sum(dReal tol)
    {

        if(empty())
            return 0.0;
        dReal ret=0.0;
        for(iterator i=begin();i<end();i++)
            ret+=exp(i->first/tol);
        return ret;
    }

    dReal stdev()
    {
       if(empty()||size()==1)
            return 0.0;
       dReal a = avg(),ret=0.0;
        for(iterator i=begin();i<end();i++)
            ret+=SQR(i->first-a);
        return sqrt(ret/(size()-1));
    }

    dReal betha_stdev(dReal tol)
    {
        if(empty()||size()==1)
            return 0.0;
        dReal exp_avg = exp_sum(tol)/size(), ret=0.0;
        for(iterator i=begin();i<end();i++)
            ret+=SQR(exp(i->first/tol)-exp_avg);
        return sqrt(ret/(size()-1));

    }

    dReal var()
    {
        if(empty())
            return 0.0;
        dReal a=avg(),ret=0.0;
        for(iterator i=begin();i<end();i++)
            ret+=SQR(i->first-a);
        return ret/size();
    }

    bool new_score(int index, dReal score)
    {
         for(iterator i=begin();i<end();i++)
             if(i->second==index)
             {
                 if(i->first>score)
                     return false;
                 i->first = score;
                 return true;
             }

         iScore tmp(score,index);
         push_back(tmp);
         return true;
    }

    int roulette_wheel(dReal tol)
    {
        dReal marker = sto.Random(),sum=0.0, esum =exp_sum(tol);
        for(iterator i=begin();i<end();i++)
        {
            sum+=exp(i->first/tol)/esum;
            if(sum>=marker)
                return i->second;
        }
        return -1;
    }

    void free()
    {
        while(!empty())
            pop_back();
    }
    void print()
    {
        for(int i=0;i<4;i++)
            cout<<"\nIndex"<<' '<<"Scores"<<' ';

        for(int i=0;i<size();i++)
        {
            if(i%4)
                cout<<endl;
            else
                cout<<' ';
            cout<<at(i).second<<' '<<at(i).first;
        }
        cout<<endl;
    }
};

extern dsFunctions fn;
extern int WORLDTYPE;
extern int OUTPUTREDIRECT;
extern int BALL;
extern int VISUAL;
extern dMatrix3 IDENTITY;
extern dWorldID world;
extern dJointGroupID contactgroup;
extern dSpaceID globalspace;

string replace(Animat *elim, Animat *A, Animat *B);
bool is_colliding(Animat *elim);

class animCapsule
//A class with singleton pattern
{
    static long int tot_time;
    static long int time_limit;
    static dReal tolerance, min_height, z_stretch;
    static int generation,anim_ind,exp_ind,failure;
    static bool is_constructed;
    static animCapsule *instance;
    static averager scores[2];
    static Animat temp_anim[3];
    static Animat *curAnim;
public:
    static bool once;
    static ofstream results;
    static void init()
    {
        animCapsule::is_constructed=false;

        results.open("results.csv",ios::app);

        DIR *dir=opendir(CREATURES_DIR);
        if(dir)
        {

            dirent *ent;
            int maxgen=0,maxnum=INSTANCE_NO;
            while(ent=readdir(dir))
                if(ent->d_type==DT_REG)//regular file
                    if(strstr(ent->d_name,".dat"))
                    {
                        string s(ent->d_name);
                        int gen = atoi(s.substr(s.find('g')+1).c_str()),
                            num = atoi(s.substr(s.find('-')+1).c_str());
                        if(maxgen<gen)
                        {
                            maxgen=gen;
                            maxnum=INSTANCE_NO;
                        }
                        if(gen==maxgen)
                            if(maxnum<num && num%PARALLELISM_SIZE==INSTANCE_NO)
                                maxnum=num;
                    }
                closedir(dir);

                generation = maxgen;
                anim_ind = maxnum-PARALLELISM_SIZE;

                if(generation)
                    while(!read_scores()&&generation)
                    {generation--;anim_ind-=PARALLELISM_SIZE;}

                init_params();

                printf("\ngeneration: %d anim_ind: %d \n\n",generation,anim_ind);
                results<<endl;
        }
    }

    static bool read_scores()
    {
        for(int i=0;i<2;i++)
            while(!scores[i].empty())scores[i].pop_back();

        for(int i=(generation)?0:1;i<POPULATION_SIZE;i++)
        {
            if(!temp_anim[0].read(generation-1,i))
            {
                if(i%PARALLELISM_SIZE==INSTANCE_NO)
                {
                    anim_ind = i;
                    return false;
                }
                else
                {
                    i--;
                    sleep(1);
                }
            }
            else
                scores[0].new_score(i,temp_anim[0].avg_score);
        }
        return true;

    }

    static void init_params()
    {
        dReal last_tolerance;
        if(generation)
        {//Read the last tolerance (temperature of Boltzmann selection) from the file
            ifstream tolerance_file;
            tolerance_file.open("tolerance.txt",ios::in);
            if(tolerance_file.is_open())
            {
                dReal x;
                int i;
                while(!tolerance_file.eof())
                {
                    tolerance_file>>i;
                    tolerance_file>>x;
                    printf("\nReading tolerance %d, it is %f", i,x);
                    if(i==generation-1)break;
                }
                if(i<generation-1)
                {
                    cout<<"\nError: cannot reach tolerance "<<generation<<endl;
                    exit(-1);
                }
                else last_tolerance = x;
                tolerance_file.close();
            }
            else
            {
                cout<<"\nError: cannot open tolerance.txt file.";
                exit(-1);
            }
            tolerance = last_tolerance /(1+0.2*last_tolerance/scores[0].betha_stdev(last_tolerance));
        }
        else
            tolerance = 15;
        ofstream tolerance_file;
        tolerance_file.open("tolerance.txt",ios::app);
        if(tolerance_file.is_open())
        {
            tolerance_file<<generation<<TAB<<tolerance<<endl;
            tolerance_file.close();
        }

        time_limit = 7500 + generation*500;
        if(generation)

        min_height = generation*.02;
        z_stretch = 0.15*generation + 0.5;
        Limb::DAMAGE_THRESHOLD = pow(generation-100,2)*1e-5+1e-6;
    }

    static void set(int gen,int n)
    {
        generation = gen;
        anim_ind = n;
        init_params();
    }

    static void load()
    {//for seeing one creature, not in the GA
        curAnim = temp_anim;
        if(!curAnim->read(generation,anim_ind))
        {cout<<"\nLoad error";exit(-1);}
        curAnim->generate(0,0,0);
        curAnim->displayRepres();
        curAnim->pushBehindXVert(0);
    }

    ~animCapsule()
    {
        if(is_constructed)
            delete instance;
    }

    static animCapsule *getInstance()
    {
        if(is_constructed)
            return instance;
        instance = new animCapsule();
        is_constructed=true;
        return instance;
    }

    static bool creature_exists(int gen, int num)
    {
        char fname[20];
        name_file(fname,num,generation);
        FILE *f=fopen(fname,"r");
        bool ret = f;
        if(f)fclose(f);
        return ret;
    }

    static void generate_next()
    {
        results<<time(0)<<TAB;
        curAnim = temp_anim;
        if(!curAnim->scoreVector.empty())
                curAnim->reset();
        if(!generation && anim_ind<POPULATION_SIZE)
        {//generation zero: randomly generated
            do
            {
                anim_ind+=PARALLELISM_SIZE;
            }while(creature_exists(generation,anim_ind));

            do
            {
                curAnim->randGenome();
                curAnim->checkGenome();
            }while(is_colliding(curAnim));

            results<<generation<<TAB<<anim_ind<<TAB;
            results<<"NULL";//current score
            results<<TAB<<"NULL"<<TAB<<"NULL";//parent one's name and score
            results<<TAB<<"NULL"<<TAB<<"NULL"<<TAB;//parent two's name and score
            results<<"NULL"<<TAB;//Method of reproduction
        }
        else
        {//generation one onward: reproduced
            do
            {
                anim_ind+=PARALLELISM_SIZE;
            }while(creature_exists(generation,anim_ind));

            if(!generation||anim_ind>=POPULATION_SIZE)
            {
                generation++;
                if(read_scores())
                {
                    init_params();
                    anim_ind=INSTANCE_NO;
                }
                else
                   do
                   {
                        generation--;
                   }while(!read_scores()&&generation);
            }


            if(anim_ind >= POPULATION_SIZE-ELITE_SIZE)
                sort(scores[0].begin(),scores[0].end());

            Animat *p1,*p2;
            int p1_index,p2_index;
            string repro_out;

            if(anim_ind < POPULATION_SIZE-ELITE_SIZE)
            {
                do
                {
                    p1_index = scores[0].roulette_wheel(tolerance),
                    p2_index = scores[0].roulette_wheel(tolerance);
                    temp_anim[1].read(generation-1,p1_index);
                    temp_anim[2].read(generation-1,p2_index);

                    p1 = temp_anim+1;
                    p2 = temp_anim+2;
                    failure=0;
                    do
                    {
                        curAnim->reset();
                        repro_out+=replace(curAnim,p1,p2);
                        failure++;
                        if(failure>10)break;
                    }while(is_colliding(curAnim));

                }while(failure>10);//To avoid clogging with headless descendants
            }

            else
            {//Choose elite from the last generation and put them into the test
                p1_index = p2_index = scores[0][anim_ind].second;
                temp_anim[0].read(generation-1,p1_index);
                p1 = p2 = curAnim = temp_anim;
                repro_out = "exact copy";
            }

            results<<generation<<TAB<<(anim_ind)<<TAB;
            results<<curAnim->avg_score;//current score
            results<<TAB<<p1_index<<TAB<<p1->avg_score;
            //parent one's index and score
            results<<TAB<<p2_index<<TAB<<p2->avg_score<<TAB;
            //parent two's index and score
            results<<repro_out<<TAB;
            strcpy(curAnim->parent[0],p1->name);
            strcpy(curAnim->parent[1],p2->name);
        }

        curAnim->generate(0,0,0);
        curAnim->displayRepres();
        curAnim->pushBehindXVert(0);
    }

    static void simLoop(int pause)
    {
    // this is the loop function to be called when performing graphical
        // simulations (see readbest.cpp and visual.cpp for examples)
        //
        // It draws everything and calls doWorld()
        int i, j;
        i = 0;

      if(curAnim)for (j=0; j < curAnim->nblimbs(); j++)
      {
          float T = 1.0/(curAnim->limbs[j].damage*.01+1);
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
      else if(!once)
        generate_next();
      else
          load();

      if(tot_time>time_limit)
          timer_ends();


      doWorld(pause, STEP,false);
    }

    static void timer_ends()
    {
        cout<<"generation "<<generation<<" anim "<<anim_ind<<" expriment "<<exp_ind<<") score:"<<curAnim->newScore(z_stretch)<<"\n";
        curAnim->motionSum.print("motionSum");
        std::cout<<"damage: "<<curAnim->damage()<<endl;
        results<<curAnim->score()<<TAB;

        curAnim->remove();
        exp_ind++;

        if(exp_ind==NEXPRMNTS)
        {
            cout<<"average score:"<<curAnim->averageScore()<<endl;
            if(((generation || (!generation&&curAnim->avg_score>MIN_SCORE))||curAnim->liked>0)&&curAnim->liked>=0)
            {
                results<<failure<<TAB<<curAnim->averageScore()<<endl;
                failure=0;
                char fname[20];
                name_file(fname,anim_ind,generation);
                curAnim->save(fname);
            }
            else
            {
                results<<failure<<TAB<<curAnim->averageScore()<<"FAILED"<<endl;
                anim_ind-=PARALLELISM_SIZE;failure++;
            }

            exp_ind=0;
            if(!once)
                generate_next();
            else
                exit(-1);
        }
        tot_time = 0;
        curAnim->generate(0,0,0);
        curAnim->pushBehindXVert(0);
    }


    static void doWorld (int pause, dReal step, int fast)
    {
        // perform a cycle of the simulation.

        dSpaceCollide (globalspace,0,&nearCallback);

        if (!pause&&curAnim)
        {
            tot_time++;
            // perform a neural update cycle
            if (curAnim->alive)
            {
                curAnim->actuate();
                if(tot_time>100)curAnim->update(step);
            }

            else
            {
                mydie(
                "Damn ! Animat registered but not alive in doWorld !\n");
            }

            if (fast)
                dWorldQuickStep(world, step); // quick, default
            else
                dWorldStep (world,step); // slower, more precise
        }
        dJointGroupEmpty (contactgroup);
    }


    static void key_command(int cmd)
    {
        if(curAnim)
        {
            if(cmd=='s'||cmd=='S')
            {
                char fname[20];
                  name_file(fname,anim_ind,generation);
                  curAnim->save(fname);
            }
            if(cmd=='l'||cmd=='L')
            {
                curAnim->liked++;
                cout<<"\nLiked: "<<curAnim->liked<<endl;
            }
            if(cmd=='d'||cmd=='D')
            {
                curAnim->liked--;
                cout<<"\nLiked: "<<curAnim->liked<<endl;
            }
        }
    }
};

Animat animCapsule::temp_anim[3];
dReal   animCapsule::tolerance=0.0,
        animCapsule::min_height=0.0,
        animCapsule::z_stretch=1;
averager animCapsule::scores[2];
long int animCapsule::tot_time=0,animCapsule::time_limit=5000;
bool animCapsule::is_constructed=false;
animCapsule *animCapsule::instance;
Animat *animCapsule::curAnim=0;
int animCapsule::anim_ind=INSTANCE_NO-PARALLELISM_SIZE,
    animCapsule::exp_ind=0,
    animCapsule::generation=0,
    animCapsule::failure=0;
ofstream animCapsule::results;
bool animCapsule::once=false;

extern int NOACTUATE;

Animat *curAnim;
long int efforts=0;
int failure[NEXPRMNTS+1];
bool do_generate = true;


int main (int argc, char **argv)
{
    BALL = 0;
    for(int i=0;i<=NEXPRMNTS;i++)
        failure[i]=0;
    WORLDTYPE = FLATWORLD;
    OUTPUTREDIRECT = NULL;
    initWorld();
    fn.step = &animCapsule::simLoop;
    fn.command= &animCapsule::key_command;
    if(argc<2)
        animCapsule::init();
    else
    {
        animCapsule::once = true;
        animCapsule::set(atoi(argv[1]),atoi(argv[2]));
        if(argc>3)
            Limb::DAMAGE_THRESHOLD = atof(argv[3]);
    }
    VISUAL=1;
    dsSimulationLoop (argc,argv,352,288,&fn);
    destroyWorld();
    animCapsule::results.flush();
    animCapsule::results.close();
}

// replaces elim's genome with either a recombination between A and B, or a
// mutated version of A or B.
string replace(Animat *elim, Animat *A, Animat *B)
{
    stringstream ostr;

    start:
    switch (sto.IRandomX(0,3))
    {
        case 0: myprintf("Cross - \n");
                if (sto.Bernoulli(0.5))
                {
                    elim->copyFrom(A);
                    elim->crossWith(B);
                    ostr<<"Cross 2 on 1"<<COMMA;
                }
                else
                {
                    elim->copyFrom(B);
                    elim->crossWith(A);
                    ostr<<"Cross 1 on 2"<<COMMA;
                }
                break;
        /*case 1: myprintf("Graft - \n");
                if (sto.Bernoulli(0.5))
                {
                    elim->copyFrom(A);
                    if(!elim->graftWith(B))
                        goto start;
                    else
                        ostr<<"Graft 2 on 1"<<COMMA;
                }
                else
                {
                    elim->copyFrom(B);
                    if(!elim->graftWith(A))
                        goto start;
                    else
                        ostr<<"Graft 1 on 2"<<COMMA;
                }
                break;*/
        case 1: myprintf("Pick - \n");
                if (sto.Bernoulli(0.5))
                {
                    elim->copyFrom(A);
                    elim->pickFrom(B);
                    ostr<<"Pick 2 on 1"<<COMMA;
                }
                else
                {
                    elim->copyFrom(B);
                    elim->pickFrom(A);
                    ostr<<"Pick 1 on 2"<<COMMA;
                }
                break;
        case 2: myprintf("Mutate A - \n");
                elim->copyFrom(A);
                elim->mutate();
                elim->mutate();
                ostr<<"Mutate 1"<<COMMA;
                break;
        case 3: myprintf("Mutate B - \n");
                elim->copyFrom(B);
                elim->mutate();
                elim->mutate();
                ostr<<"Mutate 2"<<COMMA;
                break;
    }

    elim->mutate();
    elim->reassignBadConns();

    elim->checkGenome();
    elim->checkConnections();
    return ostr.str();
}

bool is_colliding_tests(Animat *elim)
{
    NOACTUATE = 1;
    elim->generate(-50,0,0);


    if (elim->nblimbs() >= MAXLIMBS)
    {
	myprintf("Too big !\n");
	return 1;
    }

    bool head = false;
    for(int i=0;i<elim->nblimbs();i++)
        if(elim->limbs[i].type==Limb::HEAD)
        {
           if(head)return true; //multi-headed
            head = true;
        }
    if(!head)//headless
    {
        cout<<"limbs: "<<elim->nblimbs()<<" - no head!\n";
        return true;
    }

    if (elim->test_for_intra_coll())
    {
	myprintf("Self-Collided - 1st pass!\n");
	return 1;
    }
    if(elim->limbs[0].id)animCapsule::doWorld(0, STEP, true);
    if (elim->test_for_intra_coll())
    {
	myprintf("Self-Collided - 2nd pass!\n");
	return 1;
    }

    return false;
}

bool is_colliding(Animat *elim)
{
    bool ret = is_colliding_tests(elim);
    elim->remove();
    NOACTUATE=0;
    return ret;
}

