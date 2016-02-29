#include "animat.h"
#include "Neuron.h"
#include "Ball.h"
#include "definitions.h"
#include "limb.h"
#include "stocc.h"
#include <algorithm>
#include <iostream>

#define SQR(x) x*x

double ss[3]={0.1,0.1,0.1};

enum SensorTypes {ACCELX,ACCELY,ACCELZ,ANGACCELX,ANGACCELY,ANGACCELZ} SENSORTYPES[NBSENSORTYPES] = {ACCELX,ACCELY,ACCELZ,ANGACCELX,ANGACCELY,ANGACCELZ};
const char *SensorName[NBSENSORTYPES]={"ACCELX","ACCELY","ACCELZ","ANGACCELX","ANGACCELY","ANGACCELZ"};
int WORLDTYPE = -1;
dReal DAMAGETABLE[REGISTSIZE][REGISTSIZE];
int FREEZE = 0;
int AIRCOLLISIONS = 0;
extern int BOARD;
extern StochasticLib1 sto;
int DISABLESENSORS = 0;
int VISUAL = 0;
int NOACTUATE = 0;
int BALL = 0;
extern int CORRIDOR;
extern int WALLS;
extern int OUTPUTREDIRECT;
dReal GRAVCORR = 4.0 / (dReal) WORLDRADIUS;
extern dWorldID world;
dMatrix3 IDENTITY;
extern dSpaceID globalspace;
dSpaceID alternatespace; // only used within collision method

//  Still an awful lot of global variables !
dGeomID sphereID; // if sphere world is used
dGeomID groundID; // in case flat world is used
long int tot_time;
int numround;
dContact contact[MAXCONT];	// up to 4 contacts per box-box
extern dJointGroupID contactgroup;

dsFunctions fn;

Animat *regist[REGISTSIZE];
// First, a few utility function for displaying messages or saving them to
// disk...

extern FILE *OUTPUTFILE;

Ball  ball;

Animat::Animat()
        {
        alive = 0;
	ID = -1;
        nbLimbs=0;
        oldX=-1;oldY=-1;first_height=0;
        avg_score=0.0;
        head=0;
    }

bool Animat::read(int generation,int num)
{
    char fname[20];
    name_file(fname,num,generation);
    return read(fname);
}

bool Animat::read(const char *s,bool new_mode)
    {
	nbLimbs=0;
        FILE *f = fopen(s, "r");
        if(!f)
        {
            cout<<"\nRead Error: "<<s<<endl;
            return false;
        }

        if(!scoreVector.empty())
            reset();

        if(new_mode)
        {
            genome_saver loader;
            fread (&loader, 1, sizeof(genome_saver), f);
            loader.copy_on(this);
            strcpy(name,s);
            printf("name:%s p1:%s p2:%s score:%f\n",name,parent[0],parent[1],avg_score);
        }
        else
            fread (this->genome, 1, sizeof(genome), f);
        
	fflush(f);
	fclose(f);
        return true;
    }

void Animat::save(char *s)
{
        FILE *f=fopen(s,"r");    
        int i=0;
        char fname[20];
        strcpy(fname,s);

        while(f)
        {
            sprintf(fname,"[%d]%s",i,s);
            f = fopen(fname,"r");
            i++;
        }
        
        strcpy(name,fname);

        f = fopen(fname, "w");
	if(!f)
        {
            cout<<"\nWrite Error: "<<s<<endl;
            exit(-1);
        }
        
        genome_saver saver(genome,averageScore(),parent[0],parent[1]);

        fwrite (&saver, 1, sizeof(genome_saver), f);
	fflush(f);
	fclose(f);
    }

void Animat::reset()
{
        alive = 0;
	ID = -1;
        nbLimbs=0;
        liked = 0;
        oldX=-1;oldY=-1;avg_score=0;first_height=0;
        while(!Motion.empty())
        {Motion.pop();Height.pop();}
        while(!scoreVector.empty())
            scoreVector.pop_back();
        motionSum = vect2d(0,0);
        for(int i=0;i<MAXLIMBS;i++)
        {
            repres[i].dead=1;
            removeGene(i);
        }
        
}

void Animat::readOld(const char *s)
    {
	FILE *f = fopen(s, "r");
	fread (this, 1, sizeof(Animat), f);
	fclose(f);
    }

void Animat::saveOld(const char *s)
    {
	FILE *f = fopen(s, "w");
	fwrite (this, 1, sizeof(Animat), f);
	fclose(f);
    }

int Animat::nblimbs()
    {
	//if(nbLimbs)return nbLimbs;
	for (nbLimbs=0; nbLimbs < MAXLIMBS; nbLimbs++)
	    if (repres[nbLimbs].dead) break;

	return nbLimbs;
    }

void Animat::checkGenome()
    {
	// checks that the genome is valid - obviously incomplete.
	int nbg, i;
	if (genome[0].recur)
	{
	    displayGenomeFull();
	    mydie("Damn, genome[0] has recur !\n");
	}
	for (i=0; i < MAXGENES-1; i++)
	    if (genome[i].dead && !genome[i+1].dead)
	    {
		displayGenome();
		mydie("Damn, non-ordered genome !\n");
	    }
	for (i=1; i < MAXGENES-1; i++)
	{
	    if (genome[i].dead) break;
	    if (!genome[i].neurons[0].exists)
	    {
		displayGenome();
		mydie("ERROR ! Actuator does not exist in gene %d!\n", i);
	    }
	    if (!genome[i].neurons[2].exists)
	    {
		displayGenome();
		mydie("ERROR ! Sensor does not exist in gene %d!\n", i);
	    }

	}
	nbg = 0;
	for (i=0; i < MAXGENES-1; i++)
	{
	    if (!genome[i].dead) nbg++;
	}
	if (nbg < 2)
	{
		displayGenome();
		mydie("ERROR ! Only %d gene !\n", nbg);
	}
    }

int Animat::nbGenes()
    {
	int i=0;
	while ((i < MAXGENES) && (!genome[i].dead)) {i++;}
	return i;
    }

void Animat::displayGenome()
    {
	int i, j;
	for (i=0; i < MAXGENES; i++)
	{
	    myprintf("%d-%d ", genome[i].dead, genome[i].dadnum);
	    if (! genome[i].dead)
	    {
		myprintf("( ");
		for (j=0; j < MAXNEUR; j++)
		{
		    myprintf("ex:%d, ",genome[i].neurons[j].exists);
		    if ( !genome[i].neurons[j].exists) continue;
		    /*for (int k=0; k < MAXCONFROM; k++)
			myprintf("%d's %d (%.2f) ",
				genome[i].neurons[j].confrom[k].limb,
				genome[i].neurons[j].confrom[k].neur,
				genome[i].neurons[j].confrom[k].val);*/
		    myprintf(", ");
		}
		myprintf(") ");
	    }
	    myprintf("\n");
	}
	myprintf("\n");
    }

void Animat::displayGenomeFull()
    {
	int i, j;
	for (i=0; i < MAXGENES; i++)
	{
	    myprintf("-Lmb %d: ", i);
	    if (genome[i].dead)
		myprintf("dead\n");
	    else
	    {
		myprintf("(type: %d dad:%d, (%.3f,%.3f,%.3f) ",
			genome[i].type,genome[i].dadnum,
			genome[i].lgt, genome[i].wdt, genome[i].hgt);
		myprintf("alpha:%.3f, beta:%.3f, alprec:%.3f, betrec:%.3f, rec:%d, ",
			genome[i].alpha, genome[i].beta, genome[i].alpharec,
			genome[i].betarec, genome[i].recur);
		myprintf("termi:%d)\n", genome[i].terminal);
		myprintf("( ");
		for (j=0; j < MAXNEUR; j++)
		{
		    myprintf("neur %d: ",j);
		    if ( !genome[i].neurons[j].exists) myprintf("X, ");
		    if ( !genome[i].neurons[j].exists) continue;
                    if(genome[i].neurons[j].type<NBSENSORTYPES)
                        myprintf(SensorName[genome[i].neurons[j].type]);
                    if (genome[i].neurons[j].type == ACTUATOR)
			myprintf("actuator, ");
		    if (genome[i].neurons[j].type == SENSPROP)
			myprintf("sensprop, ");
		    if (genome[i].neurons[j].function == INVEXP)
			myprintf("invexp, ");
		    if (genome[i].neurons[j].function == SIGMOID)
			myprintf("sig, ");
		    if (genome[i].neurons[j].function == TANH)
			myprintf("mytanh, ");
		    myprintf("thres: %.2f, ",
			    genome[i].neurons[j].threshold);
		    for (int k=0; k < MAXCONFROM; k++)
		    {
			if (genome[i].neurons[j].confrom[k].exists == 0)
			    myprintf("(o) ");
			else
			{
			    myprintf("(%d:%d, %.2f, ",
				genome[i].neurons[j].confrom[k].limb,
				genome[i].neurons[j].confrom[k].neur,
				genome[i].neurons[j].confrom[k].val
				);
			    if (genome[i].neurons[j].confrom[k].rectype == RECDAD)
				myprintf("rdad-");
			    if (genome[i].neurons[j].confrom[k].rectype == RECSON)
				myprintf("rson-");
			    if (genome[i].neurons[j].confrom[k].rectype == RECSELF)
				myprintf("rself-");
			    if (genome[i].neurons[j].confrom[k].reftype == REFSYMM)
				myprintf("rsymm)");
			    if (genome[i].neurons[j].confrom[k].reftype == REFORIG)
				myprintf("rorig)");
			    if (genome[i].neurons[j].confrom[k].reftype == REFBOTH)
				myprintf("rboth)");
			}
		    }
		    myprintf(", ");
		}
		myprintf(") ");
	    }
	    myprintf("\n");
	}
	myprintf("\n");
    }

void Animat::displayRepres()
    {
	int i, j;
	for (i=0; i < MAXLIMBS; i++)
	{
	    myprintf("-Lmb %d: ", i);
	    if (repres[i].dead)
		myprintf("dead\n");
	    else
	    {
		myprintf("(dad:%d, orig:%d, lgt:%.3f, wdt:%.3f, hgt:%.3f, ",
			repres[i].dadnum, repres[i].origgene,
			repres[i].lgt, repres[i].wdt, repres[i].hgt);
		myprintf("alpha:%.3f, beta:%.3f, alprec:%.3f, betrec:%.3f, rec:%d, term:%d) ",
			repres[i].alpha, repres[i].beta, repres[i].alpharec,
			repres[i].betarec, repres[i].recur,
			repres[i].terminal);
		myprintf("( \n");
		for (j=0; j < MAXNEUR; j++)
		{
		    myprintf("neur %d: ",j);
		    if ( !repres[i].neurons[j].exists) myprintf("X, ");
		    if ( !repres[i].neurons[j].exists) continue;
		    if (repres[i].neurons[j].type == ACTUATOR)
			myprintf("ACTUATOR, ");
                    if (repres[i].neurons[j].type == SENSPROP)
			myprintf("sensprop, ");
		    if (repres[i].neurons[j].function == INVEXP)
			myprintf("invexp, ");
		    if (repres[i].neurons[j].function == SIGMOID)
			myprintf("sig, ");
		    if (repres[i].neurons[j].function == TANH)
			myprintf("mytanh, ");
		    myprintf("thres: %.2f, ",
			    repres[i].neurons[j].threshold);
		    for (int k=0; k < MAXCONFROM; k++)
		    {
			if (repres[i].neurons[j].confrom[k].exists == 0)
			    myprintf("(o) ");
			else
			{
			    myprintf("(%d:%d, %.2f, ",
				repres[i].neurons[j].confrom[k].limb,
				repres[i].neurons[j].confrom[k].neur,
				repres[i].neurons[j].confrom[k].val
				);
			    if (repres[i].neurons[j].confrom[k].rectype == RECDAD)
				myprintf("rdad-");
			    if (repres[i].neurons[j].confrom[k].rectype == RECSON)
				myprintf("rson-");
			    if (repres[i].neurons[j].confrom[k].rectype == RECSELF)
				myprintf("rself-");
			    if (repres[i].neurons[j].confrom[k].reftype == REFSYMM)
				myprintf("rsymm)");
			    if (repres[i].neurons[j].confrom[k].reftype == REFORIG)
				myprintf("rorig)");
			    if (repres[i].neurons[j].confrom[k].reftype == REFBOTH)
				myprintf("rboth)");
			}
		    }
		    myprintf(",\n ");
		}
		myprintf(") ");
	    }
	    myprintf("\n");
	}
	myprintf("\n");
    }

void Animat::compress()
    {
	int to, from, i, k, n;
	for (to=0; to < MAXGENES; to++)
	{
	    from = to+1;
	    while ((genome[to].dead) && (from < MAXGENES))
	    {
		genome[to] = genome[from];
		genome[from].dead = 1;
		// if (!genome[i].dead)  <=== may be useful ??
		for (k=0; k < MAXGENES; k++)
		    if (genome[k].dadnum == from)
			genome[k].dadnum = to;
		for (n=0; n < MAXGENES; n++)
		    for (i=0; i < MAXNEUR; i++)
			for (k = 0; k < MAXCONFROM; k++)
			    if (genome[n].neurons[i].confrom[k].limb == from)
				genome[n].neurons[i].confrom[k].limb = to;
		from++;
	    }
	    if (from >= MAXGENES) return;
	}
    }

dReal Animat::curDamageFrom(Animat *other)
    {
	// The amount of damage that this other individual is
	// currently dealing to me (i.e. in this very timestep)
	for (int me=0; me < REGISTSIZE; me++)
	{
	    if (!regist[me]) continue;
	    if (regist[me] != this) continue;
	    for (int i=0; i < REGISTSIZE; i++)
	    {
		if (!regist[i]) continue;
		if (regist[i] == this) continue;
		if (regist[i] == other)
		    return DAMAGETABLE[i][me];
	    }
	}
	return 0;
    }

void Animat::addNewGeneOld()
    {
	int i;
	i = nbGenes();
	if (i >= MAXGENES) return;
	genome[i].randBodyGene(sto.IRandomX(0,i-1));
	genome[i].dead = 0;
	makeNeuronsLimb(i);
	randConnectionsLimb(i);
    }

void Animat::addNewGene()
    {
	// Adds a new random gene to the genome
	int i, n, k, numdad, numson, num, nbsons, sons[MAXGENES];
	i = nbGenes();
	if (i >= MAXGENES) { myprintf("Can't add new gene, genome full\n");return;}
	numdad = sto.IRandomX(0,i-1);
	myprintf("numdad = %d\n", numdad);
	for (num=MAXGENES-2; num > numdad; num--)
	{
	    myprintf("pushing %d to %d\n", num, num+1);
	    genome[num+1] = genome[num];
	    for (k=0; k < MAXGENES; k++)
		if (genome[k].dadnum == num)
		    genome[k].dadnum = num+1;
	    for (n=0; n < MAXGENES; n++)
		for (i=0; i < MAXNEUR; i++)
		    for (k = 0; k < MAXCONFROM; k++)
			if (genome[n].neurons[i].confrom[k].limb == num)
			    genome[n].neurons[i].confrom[k].limb = num+1;
	}
	num = numdad + 1;
	myprintf("num = %d\n", num);
	// temporarily, num must not be a son of numdad!
	genome[num].dadnum = -1;
	nbsons=0;
	for (i=0; i < MAXGENES; i++)
	    if (genome[i].dadnum == numdad)
	    {
		sons[nbsons] = i;
		nbsons ++;
	    }
	if (nbsons)
	{
	    numson = sto.IRandomX(0,nbsons); // we either pick a son, or not
	    if (numson < nbsons) // if we do... rebranch that son from num
	    {
		myprintf("rebranching %d\n", sons[numson]);
		genome[sons[numson]].dadnum = num;
		for (i=0; i < MAXNEUR; i++)
		    for (k = 0; k < MAXCONFROM; k++)
			if (genome[sons[numson]].neurons[i].confrom[k].limb
				== numdad)
			    genome[sons[numson]].neurons[i].confrom[k].limb
				= num;
	    }

	}
	genome[num].randBodyGene(numdad);
	genome[num].dead = 0;
	makeNeuronsLimb(num);
	randConnectionsLimb(num);
	reassignBadConns();
    }

void Animat::removeGene(int a)  // DOES NOT COMPRESS THE GENOME !
    {
	// remove a gene from the genome, together with ALL ITS CHILDREN GENES,
	// without compressing the genome
	// GENOME MUST BE COMPRESSED AT SOME POINT AFTER USE
	int i;
	//myprintf("%d.",a);
	for (i=0; i < MAXGENES; i++)
	    if ((genome[i].dadnum == a)
		    && (!genome[i].dead))
		removeGene(i);
	genome[a].dead = 1;
	for (i=0; i < MAXNEUR; i++)
	    genome[a].neurons[i].exists = 0;
    }

void Animat::remove1Gene(int a)
    {
	// removes only this gene, NOT the children genes.
	// This requires reassigning the children genes to the dad of the
	// deleted  gene
	int dad, i, j, k, nbsons=0;
	int tabsons[MAXGENES];
	dad = genome[a].dadnum;
	myprintf("Removing gene %d (dad is %d)\n", a, dad);
	for(i=0; i < MAXGENES; i++)
	    if ((genome[i].dadnum == a)
		    && (!genome[i].dead))
	    {
		tabsons[nbsons] = i;
		nbsons ++;
		genome[i].dadnum = dad;
		for (j=0; j < MAXNEUR; j++)
		    for (k = 0; k < MAXCONFROM; k++)
			if (genome[i].neurons[j].confrom[k].limb
				== a)
			    genome[i].neurons[j].confrom[k].limb
				= dad;
	    }
	genome[a].dead = 1;
	for (i=0; i < MAXNEUR; i++)
	    genome[a].neurons[i].exists = 0;
	for (i=0; i < MAXNEUR; i++)
	    if (genome[dad].neurons[i].exists)
		for (j=0; j < MAXCONFROM; j++)
		    if (genome[dad].neurons[i].confrom[j].exists &&
			    (genome[dad].neurons[i].confrom[j].limb == a))
		    {
			// if conn points from deleted limb, it must be
			// reassigned. However, I want it to be
			// reassigned to point from one of the deleted limbs'
			// sons, not from
			// the dad or from limb 0 as
			// randConn might do ...  .. but what if there are no
			// new sons, i.e. the deleted limb had no child ?
			if (nbsons)
			{
			    myprintf("Trying to randomise con %d:%d:%d to a son of %d\n",
				    dad, i, j, dad);
			    int out=0;
			    do
			    {
				out=0;
				randConn(dad, i, j);
				for (int cpt=0; cpt < nbsons; cpt++)
				    if (tabsons[cpt] == genome[dad].neurons[i].confrom[j].limb)
					out = 1;
			    } while (!out);

			    //while (genome[genome[dad].neurons[i].confrom[j].limb].dadnum
				//    != dad );
			}
			else
			    randConn(dad, i, j);
		    }
	for (i=0; i < MAXNEUR; i++)
	    if (genome[0].neurons[i].exists)
		for (j=0; j < MAXCONFROM; j++)
		    if (genome[0].neurons[i].confrom[j].exists &&
			    (genome[0].neurons[i].confrom[j].limb == a))
			randConn(0, i, j);
	compress();

    }

void Animat::insertFromOld(Animat *B, int nfrom, int nto)
    {
	int i = 0, newnum, j, k;
	while ((i < MAXGENES) && (!genome[i].dead)) i++;
	if (i == MAXGENES) return;
	newnum = i;
	genome[newnum] = B->genome[nfrom];
	for (j=0; j < MAXNEUR; j++)
	{
	    for (k=0; k < MAXCONFROM; k++)
	    {
		if (genome[newnum].neurons[j].confrom[k].limb
			== genome[newnum].dadnum)
		    genome[newnum].neurons[j].confrom[k].limb = nto;
		else
		    if (genome[newnum].neurons[j].confrom[k].exists)
			genome[newnum].neurons[j].confrom[k].limb += 999;
	    }
	}
	genome[newnum].dadnum = nto;
	for (i=0; i < MAXGENES; i++)
	    for (j=0; j < MAXNEUR; j++)
		for (k=0; k < MAXCONFROM; k++)
		    if (genome[i].neurons[j].confrom[k].exists)
			if (genome[i].neurons[j].confrom[k].limb - 999
				== nfrom)
			    genome[i].neurons[j].confrom[k].limb = newnum;
	for (i=0; i < MAXGENES; i++)
	    if ((B->genome[i].dadnum == nfrom) &&
		    !(B->genome[i].dead))
		insertFromOld(B, i, newnum);
    }

void Animat::insertFromLessOld(Animat *B, int dadfrom, int dadto)
    {
	int ipoint, insfrom, j, k, l, tab[MAXGENES][2], idx=0;
	for (j=0; j < MAXGENES; j++) { tab[j][0] = -2; }
	for (insfrom=0; insfrom < MAXGENES; insfrom++)
	    if (B->genome[insfrom].dadnum == dadfrom)
	    {
		// no need to insert deads..
		if (B->genome[insfrom].dead) continue;
		j=0; while (( !genome[j].dead ) && (j <= MAXGENES)) j++;
		if (j == MAXGENES) return;
		ipoint = j;
		genome[ipoint] = B->genome[insfrom];
		tab[idx][0] = insfrom; tab[idx][1] = ipoint; idx++;
		for (j=0; j < MAXNEUR; j++)
		    for (k=0; k < MAXCONFROM; k++)
		    {
			if (genome[ipoint].neurons[j].confrom[k].limb
				== genome[ipoint].dadnum)
			    genome[ipoint].neurons[j].confrom[k].limb = dadto;
			else
			    if (genome[ipoint].neurons[j].confrom[k].limb != 0)
				genome[ipoint].neurons[j].confrom[k].limb
				    += 999;
		    }
		genome[ipoint].dadnum = dadto;
		for (l=0; l < MAXGENES; l++)
		    for (j=0; j < MAXNEUR; j++)
			for (k=0; k < MAXCONFROM; k++)
			    if (genome[l].neurons[j].confrom[k].limb -999
				    == insfrom)
				genome[l].neurons[j].confrom[k].limb
				    = ipoint;
	    }
	for (insfrom=0; insfrom < MAXGENES; insfrom++)
	    if (tab[insfrom][0] != -2)
		insertFromLessOld(B, tab[insfrom][0], tab[insfrom][1]);
    }

void Animat::checkConnections()
    {
	// Makes sure everything is OK
	int limb, neur, conn;
	for (limb=0; limb < MAXGENES; limb++)
	{
	    if (genome[limb].dead) continue;
	    for (neur=0; neur < MAXNEUR; neur++)
	    {
		if (!genome[limb].neurons[neur].exists) continue;
		//test:
		for (conn=0; conn < MAXCONFROM; conn++)
		    if (genome[limb].neurons[neur].confrom[conn].exists)
		    {
			if (genome[limb].neurons[neur].confrom[conn].neur < 0)
			{
			    mydie("Damn ! Existing conn to neg neur %d\n",
		    genome[limb].neurons[neur].confrom[conn].neur);
			}
			if (genome[limb].neurons[neur].confrom[conn].limb < 0)
			{
			    mydie("Damn ! Existing conn to neg limb %d!\n",
		    genome[limb].neurons[neur].confrom[conn].limb);
			}
		    }
	    }
	}
    }

void Animat::setImmunityTimer(int time)
    {
	for (int i=0; i < nblimbs(); i++)
	{
	    limbs[i].immunitytimer = time;
	}
    }

void Animat::resetDamages()
    {
	for (int i=0; i < nblimbs(); i++)
	{
	    limbs[i].damage = 0;
	    limbs[i].damagedone = 0;
	}
    }

dReal Animat::getTotalDamageDone()
    {
	// total amount of damage I have dealt since last call to
	// resetDamages()
	dReal sum = 0;
	for (int i=0; i < nblimbs(); i++)
	    sum += limbs[i].damagedone;
	return sum;
    }

dReal Animat::getTotalDamage()
    {
	// total amount of damage I have received since last call to
	// resetDamages()
	dReal sum = 0;
	for (int i=0; i < nblimbs(); i++)
	    sum += limbs[i].damage;
	return sum;
    }

void Animat::deleteUnconnNeurons()
    {
	// Not used
	int limb, neur, connect, conn, l2, n2, c2;
	for (limb=0; limb < MAXGENES; limb++)
	{
	    if (genome[limb].dead) continue;
	    for (neur=0; neur < MAXNEUR; neur++)
	    {
		if (!genome[limb].neurons[neur].exists) continue;
		if ((genome[limb].neurons[neur].type == SENSPROP)
		    || (genome[limb].neurons[neur].isExtSensor()))
		    continue;
		if (genome[limb].neurons[neur].type == ACTUATOR) continue;
		// if is has a conn from another existing neuron,
		// it is connected
		for (conn=0; conn < MAXCONFROM; conn++)
		    if ((genome[limb].neurons[neur].confrom[conn].exists)
			&& (
			    // hmmm syntax freaks beware..
			    genome[genome[limb].neurons[neur].confrom[conn].limb].neurons[genome[limb].neurons[neur].confrom[conn].neur].exists
			   ))
			connect = 1;
		// If another neuron has an existing conn from it, it
		// is connected
		for (l2=0; l2 < MAXGENES; l2++)
		{
		    if (genome[l2].dead) continue;
		    for (n2=0; n2 < MAXNEUR; n2++)
		    {
			if (!genome[l2].neurons[n2].exists) continue;
			for (c2=0; c2 < MAXCONFROM; c2++)
			    if ((genome[l2].neurons[n2].confrom[c2].exists)
				&& (genome[l2].neurons[n2].confrom[c2].limb == limb)
				&& (genome[l2].neurons[n2].confrom[c2].neur == neur))
				connect = 1;
		    }
		}
		if (!connect)
		{
		    myprintf("Unconnected [%d:%d] !\n",limb,neur);
		    genome[limb].neurons[neur].exists = 0;
		}
	    }
	}
    }

void Animat::copyFrom(Animat *B)
    {
	int i;
	for (i=0; i < MAXGENES; i++)
	    genome[i] = B->genome[i];
	ID = B->ID; PID1 = B->PID1; PID2 = B->PID2;avg_score=B->avg_score;

    }

void Animat::copyGenomeOnlyFrom (Animat *B)
    {
	int i;
	for (i=0; i < MAXGENES; i++)
	    genome[i] = B->genome[i];
    }

void Animat::crossWith (Animat *B)
    {
	// Simple crossover method
	// (Methode "bourrin")
	int i, CO1;
	i=0;
	CO1 = sto.IRandomX(1,nbGenes());
	for (i=CO1; i < MAXGENES; i++)
	    genome[i] = B->genome[i];
    }

void Animat::reassignBadConns()
    {
	// Exact contract:
	// any existing conn from an existing limb which is non-adjacent or
	// not root limb OR NOT SAME LIMB, or from a non-existing limb,
	//  is randomly
	// reassigned to a legitimate source

	int li, ne, co, lifrom;
	dReal weight;
	Neuron *tmpn;
	for (li=0; li < MAXGENES; li++)
	    for (ne=0; ne < MAXNEUR; ne++)
		for (co=0; co < MAXCONFROM; co++)
		{
		    if (genome[li].dead) continue;
		    tmpn = &genome[li].neurons[ne];
		    if (tmpn->confrom[co].exists)
		    {
			lifrom = tmpn->confrom[co].limb;
			weight = tmpn->confrom[co].val;
			if ( (lifrom != genome[li].dadnum) &&
				(genome[lifrom].dadnum != li) &&
				(lifrom != li) &&
				(li != 0) &&
				(lifrom != 0))
			{
			    myprintf("Reassigning %d:%d con %d ", li, ne, co);
			    myprintf("from %d:%d to ", lifrom, tmpn->confrom[co].neur);
			    randConn(li, ne, co);
			    myprintf("%d:%d\n", tmpn->confrom[co].limb, tmpn->confrom[co].neur);
			}
			if (genome[lifrom].dead)
			{
			    myprintf("Reassigning %d:%d con %d ", li, ne, co);
			    myprintf("from %d:%d (%d dead) to ", lifrom, tmpn->confrom[co].neur,
				    lifrom);
			    randConn(li, ne, co);
			    myprintf("%d:%d\n", tmpn->confrom[co].limb, tmpn->confrom[co].neur);
			}
			tmpn->confrom[co].val = weight;
		    }
		}

    }

int Animat::insertFrom(Animat *B, int from, int dadto)
    {
	// Inserts a sub-tree from B's limb graph into my own
	// More precisely: insert a copy of limb "from" off animat B (together
	// with all its sub-limbs) to my own limb "dadto"
	// Tricky code.
	int idx, n, i, neu, con, orignewlimb;
	int copyfrom[MAXGENES];
	// the easy part: creating an array to associate limbs to be copied
	// (from B's genome)
	// to places where they should be copied (in my genome).
	// copyfrom[placeinthegenome] = numoflimbinBtobecopiedhere if a limb
	// is to be copied, -4 otherwise
	idx = 0;
	for (n=0; n < MAXGENES; n++) copyfrom[n] = -4;
	while ((idx < MAXGENES) && (!genome[idx].dead)) idx ++;
	orignewlimb = idx;
	//myprintf("Inserting from orig. idx = %d\n", idx);
	if (idx >= MAXGENES) return -1;
	for (n=0; n < MAXGENES; n++)
	{
	    if (B->isInSublimbGenome(n, from))
	    {
	//myprintf("In B, %d is in sublimb originating in %d !\n", n, from);
	//myprintf("In B, %d's dad is  %d !\n", n, B->genome[n].dadnum);

		copyfrom[idx] = n;
		do { idx ++ ; } while ((idx < MAXGENES) && (!genome[idx].dead));
		if (idx >= MAXGENES) break;
	    }
	}

	// Array made, now the tough part
	for (n=0; n < MAXGENES; n++)
	    if (copyfrom[n] != -4)
	    {
		//printf("gene %d is copied from B's gene %d\n", n, copyfrom[n]);
		genome[n] = B->genome[copyfrom[n]];

		// should only be -4 for first limb of newly inserted subtree,
		// i.e. :
		if (copyfrom[n] == from)
		    genome[n].dadnum = dadto;
		else
		{
		    for (i=0; i < MAXGENES; i++)
			if (copyfrom[i] == B->genome[copyfrom[n]].dadnum)
			{
			    genome[n].dadnum = i;
			    break;
			}
		    if (i >= MAXGENES)
		    {
			myprintf("Damn ! Couldn't find %d's dad copy\n", n);
			myprintf("copyfrom[%d] = %d\n",
				n, copyfrom[n]);
			myprintf("Dad of copyfrom[%d] is %d\n", n,
			B->genome[copyfrom[n]].dadnum);
		    for (i=0; i < MAXGENES; i++)
			if (copyfrom[i] == B->genome[copyfrom[n]].dadnum)
			    myprintf("Found cf[%d] = %d\n", i, copyfrom[i]);
			//mydie("i >= MAXGENES\n");
                    return -1;
		    }
		}


		//modifying neural info...
		for (neu = 0; neu < MAXNEUR; neu++)
		    for (con=0; con < MAXCONFROM; con++)
			if (genome[n].neurons[neu].confrom[con].exists)
			{
			    if (genome[n].neurons[neu].confrom[con].limb < 0)
                                return -1;
				//mydie("Damn, Existing con from neg limb\n");

			    //myprintf("Treating %d:%d-%d\n", n, neu, con);

			    if (genome[n].neurons[neu].confrom[con].limb
				    == B->genome[from].dadnum)
				genome[n].neurons[neu].confrom[con].limb = dadto;
			    else
			    {
				for (i=0; i < MAXGENES; i++)
				    if (copyfrom[i] ==
					    genome[n].neurons[neu].confrom[con].limb)
					genome[n].neurons[neu].confrom[con].limb = i;
			    }
			}
	    }

	// if the addition results in the new child having the same
	// orientation as a pre-existing child, or has orientation 0,0 in
	// a recursified limb, we try to re-orientate it
	int k = 0;
	int colli = 0;
	do
	{
	    colli = 0;
	    for (i=0; i < MAXGENES; i++)
	    {
		if (((genome[orignewlimb].alpha == 0)
			&& (genome[orignewlimb].beta == 0)
			&& (genome[orignewlimb].recur > 0))
			||
		 ((genome[i].dadnum == dadto)
			&& (genome[i].alpha == genome[orignewlimb].alpha)
			&& (genome[i].beta == genome[orignewlimb].beta)))
		{
		    colli = 1;
		    genome[orignewlimb].alpha = dReal (sto.IRandomX(0,7) - 4) / 8.0; // [-4/8..3/8]
		    genome[orignewlimb].beta = dReal (sto.IRandomX(0,7) - 4) / 8.0; // [-4/8..3/8]
		}
	    }
	    k++;
	} while (colli && (k < 100));

// WHAT IF LIMB NOT FULLY GRAFTED ?
	// Well then the conn will be randomly reassigned
	// Note that in our experiments, a creature which reaches the
	// max number of limbs is discarded, so any creature for
	// which limbs are not fully grafted due to reaching max limit will be
	// discarded as well
	for (i=0; i < MAXGENES; i++)
	{
	    if (genome[i].dadnum == -4)
	    {
		myprintf("Damn ! Incorrect dad reassignment in grafting !\n");
		myprintf("Error at gene %d with dadnum -4\n", i);
		myprintf("copyfrom[%d] = %d, with dadnum(in B) %d, copyfrom %d\n",
			i, copyfrom[i], B->genome[copyfrom[i]].dadnum,
			copyfrom[B->genome[copyfrom[i]].dadnum]);
		//mydie("Incorrect dad reassignment in insertFrom\n");
                return -1;
	    }
	    if (genome[i].dadnum >= i)
	    {
		myprintf("Damn ! Incorrect dad ordering !\n");
		myprintf("Error at gene %d with dadnum %d\n", i,
			genome[i].dadnum);
		myprintf("copyfrom[%d] = %d, with dadnum(in B) %d, copyfrom %d\n",
			i, copyfrom[i], B->genome[copyfrom[i]].dadnum,
			copyfrom[B->genome[copyfrom[i]].dadnum]);
                return -1;
		//mydie("Incorrect dad ordering in insertFrom\n");
	    }

	}
	checkGenome();
	return(orignewlimb);
    }

void Animat::pickFrom(Animat *B)
    {
	int CO1, CO2;
	CO1 = sto.IRandomX(0,nbGenes()-1);
	CO2 = sto.IRandomX(1, B->nbGenes());
	myprintf("Picking at CO1 %d CO2 %d\n", CO1, CO2);
	displayGenome();
	B->displayGenome();
	insertFrom (B, CO2, CO1);
    }

bool Animat::graftWith(Animat *B)
    {
	// The other, non-trivial crossover operator.
	// Replace a certain subtree from my limb graph with a sub-tree from
	// the limb graph of B

	// How graftWith works: Find a subtree. Delete it. Compress. Insert new
	// subtree with insertFrom, keeping the same dad as the old, deleted
	// subtree.
	//
	// Together with insertFrom, this is the trickiest part of the code.
	int nb1, nb2, CO1, CO2, insertpoint, i, j, k, found, orignewlimb;
	Neuron neurtemp[MAXNEUR];
	nb1 = 1; nb2 = 1; i = 0;
	i=0;
	CO1 = sto.IRandomX(1,nbGenes());
	found =0;
	CO2 = sto.IRandomX(1,B->nbGenes()); // CO2 must not nec. be a dad !
	myprintf(" CO1: %d, CO2: %d\n", CO1, CO2);
	insertpoint = genome[CO1].dadnum;
	if (insertpoint > CO1) mydie("ERROR ! - Non-ordered genome !\n");
	for (i=0; i < MAXNEUR; i++)
	    neurtemp[i] = genome[insertpoint].neurons[i];
	for (j=0; j < MAXNEUR; j++)
	    for (k=0; k < MAXCONFROM; k++)
		if (genome[insertpoint].neurons[j].confrom[k].limb == CO1)
		    genome[i].neurons[j].confrom[k].limb = 997;
	removeGene(CO1);
	compress(); // compress takes care of updating conns referring to limbs moved
	orignewlimb = insertFrom (B, CO2, insertpoint);
	if (orignewlimb == -1)return false;
	for (i=0; i < MAXGENES; i++)
	    for (j=0; j < MAXNEUR; j++)
		for (k=0; k < MAXCONFROM; k++)
		  if (genome[i].neurons[j].confrom[k].limb == 997)
		      genome[i].neurons[j].confrom[k].limb = orignewlimb;
        return true;
    }


void Animat::clearNeur(int limb, int num)
    {
	// clears up a neuron's info
	int i;
	genome[limb].neurons[num].exists = 0;
	for (i=0; i < MAXCONFROM; i++)
	    genome[limb].neurons[num].confrom[i].exists = 0;
    }

void Animat::delNeur(int limb, int num)
    {
	// delete the neuron, AND any connection in the genome that points to
	// it
	int i, j, k;
	for (i=0; i < MAXGENES; i++)
	    for (k=0; k < MAXNEUR; k++)
		for (j=0; j < MAXCONFROM; j++)
		    if ((genome[i].neurons[k].confrom[j].limb == limb)
			&& (genome[i].neurons[k].confrom[j].neur == num))
			genome[i].neurons[k].confrom[j].exists = 0;
	genome[limb].neurons[num].exists = 0;
	for (i=0; i < MAXCONFROM; i++)
	    genome[limb].neurons[num].confrom[i].exists = 0;
    }


void Animat::randConn(int limb, int neur, int conn)
    {
	// generate random, valid values for this connection of this neuron of
	// this limb.
	static int tab[MAXGENES*MAXNEUR][2];
	int nbchoice=0;
	int num, i, j;
	//myprintf("in randConn(%d, %d, %d)\n", limb, neur, conn);
	if (genome[limb].dead) return;
	for (i=0; i < MAXGENES; i++)
	{
	    if (genome[i].dead) continue; // not dead ! causes infinite loop in rem1gene !
	    for (j=0; j < MAXNEUR; j++)
	    {
		if (genome[i].neurons[j].exists == 0) continue;
		if (genome[i].neurons[j].type == ACTUATOR) continue; // ?...
		if ((genome[limb].dadnum == i) || (genome[i].dadnum == limb)
			|| (i == 0) || (limb == 0) // all conns from/to limb 0
			|| ((i == limb) /*&& (j != neur)*/))

		{
		    if (nbchoice >= MAXNEUR * MAXGENES)
		    {
			mydie("Damn ! Too many choices !\n");
		    }
		    tab[nbchoice][0] = i;  // (i,j) is a valid choice for connection
		    tab[nbchoice][1] = j;  // (i,j) is a valid choice for connection
		    nbchoice++;
		}
	    }
	}
	if (nbchoice == 0)
	{
	    char str[100];
	    sprintf(str,"couldntfindforconn%d:%d:%d.dat", limb, neur, conn);
	    save(str);
	    // Should never happen if cnx to same limb is authorized (we must
	    // authorize it for animats composed of only limb 0)
	    genome[limb].neurons[neur].confrom[conn].exists = 0;
			mydie("Damn ! Couldn't find any neuron to connect from!\n");
	    //myprintf("%d %d Couldn't find any neuron to connect to\n",limb,neur);
	    //return;
	}
	num = sto.IRandomX(0,nbchoice-1);
	genome[limb].neurons[neur].confrom[conn].limb = tab[num][0];
	genome[limb].neurons[neur].confrom[conn].neur = tab[num][1];
	genome[limb].neurons[neur].confrom[conn].val = (sto.IRandomX(0,60)) -30;  // -3..3
	genome[limb].neurons[neur].confrom[conn].val /= 30.0;

	i = sto.IRandomX(0,100);
	if (i < 75) genome[limb].neurons[neur].confrom[conn].reftype = REFBOTH;
	else if (i < 87) genome[limb].neurons[neur].confrom[conn].reftype = REFORIG;
	else genome[limb].neurons[neur].confrom[conn].reftype = REFSYMM;

	i = sto.IRandomX(0,100);
	if (i < 33) genome[limb].neurons[neur].confrom[conn].rectype = RECSELF;
	else if (i < 66) genome[limb].neurons[neur].confrom[conn].rectype = RECDAD;
	else genome[limb].neurons[neur].confrom[conn].rectype = RECSON;
	/*if (limb == 0)
	    printf("Limb 0 neur %d conn %d is from %d:%d\n",
		    neur, conn,
		    genome[limb].neurons[neur].confrom[conn].limb,
		    genome[limb].neurons[neur].confrom[conn].neur);  */
    }


bool Animat::mutate()
    {
	// The mutation operator
	int num, i, j;
	Neuron *tmpneur;
	Animat tmpAnimat;
	tmpAnimat.copyFrom(this);
	/*if (random() % 100 < PROBAMUT)
	    for (i=0; i < 10; i++)
	    {
		removeGene(1 + random() % (nbGenes() - 1));
		compress();
		if (nbGenes() >= MINGENES) break;
		else copyFrom(&tmpAnimat);
	    }*/
	// Adding or deleting genes..
	if (sto.IRandomX(0,100) < PROBAMUT)
	    if (nbGenes() > MINGENES)
	    {
		for (i=0; i < 10; i++)
		{
		    remove1Gene(sto.IRandomX(1,nbGenes()));
		    compress();
		    if (nbGenes() >= MINGENES) break;
		    else copyFrom(&tmpAnimat);
		}
	    }
	if (sto.IRandomX(0,100) < PROBAMUT)
		addNewGene();
	// Turn one recursive loop in the genome, if any, into separate
	// (identical) genes - source of duplication (and hopefully later
	// exaptation)
	if (sto.IRandomX(0,100) < PROBAMUT)
	        dev1RecInGenome();
	//deleteUnconnNeurons();
	checkGenome(); // you never know.
	// Neural mutations...
	for (num=0; num < MAXGENES; num++)
	{
	    if (genome[num].dead) break;
	    for (i=0; i < MAXNEUR; i++)
	    {
		// delete or create neurons
		tmpneur = &genome[num].neurons[i];
		// you can't delete proprioceptors and actuators,
		// or (if sensors are imposed in trunk) the first 2 neurons of
		// limb 0
		if ((sto.IRandomX(0,100) < PROBAMUT)
			&& (tmpneur->type != ACTUATOR)
			&& (tmpneur->type != SENSPROP)
			&& ((!ENFORCESENSORSINTRUNK) ||
			    ((num != 0) || (i > 1))))
		{
		    //myprintf("Flip Neur %d Gene %d..", i, num);
		    /*if ((i != 0) && (i != 1) && (i != 2) && (i != 3) &&
			    (i != 4))*/
			    //((num != 0) || (i != 4)))
		    if (tmpneur->exists) tmpneur->exists = 0;
		    else
		    {
			tmpneur->exists = 1;
			tmpneur->randVals();
			randConnectionsNeur(num, i);
			if (sto.IRandomX(0,100) < PROBANEURSENS)
			{
			    int val;
			    val = sto.IRandomX(0, NBSENSORTYPES-1);
			    tmpneur->type = SENSORTYPES[val];

			    /*val = random() % 4;
			    if (val == 0)
				tmpneur->type = SENSBALLX;
			    if (val == 1)
				tmpneur->type = SENSBALLY;
			    if (val == 2)
				tmpneur->type = SENSCLOSESTANIMY;
			    if (val == 3)
				tmpneur->type = SENSCLOSESTANIMX;*/
			}
		    }

		}
		// changing neuron data...
		if (tmpneur->exists)
		{
		    // changing neuron type, if allowed...
		    if ((sto.IRandomX(0,100) < PROBAMUT)
			    && (tmpneur->type != ACTUATOR)
			    && (tmpneur->type != SENSPROP)
			    && ((!ENFORCESENSORSINTRUNK) ||
				((num != 0) || (i > 1))))
		    {
			if (tmpneur->isExtSensor())
			{
			    tmpneur->type = INTER;
			}
			else
			{
			    int val = sto.IRandomX(0, NBSENSORTYPES-1);
			    tmpneur->type = SENSORTYPES[val];
			    /*int val;
			    val = random() % 2;
			    if (val == 0)
				tmpneur->type = SENSBALLX;
			    if (val == 1)
				tmpneur->type = SENSCLOSESTANIMY;*/
			    /*val = random() % 4;
			    if (val == 0)
				tmpneur->type = SENSBALLX;
			    if (val == 1)
				tmpneur->type = SENSBALLY;
			    if (val == 2)
				tmpneur->type = SENSCLOSESTANIMY;
			    if (val == 3)
				tmpneur->type = SENSCLOSESTANIMX;*/
			}
		    }
		    if (sto.IRandomX(0,100) < PROBAMUT)
		    {
			tmpneur->threshold /= MAXTHRES;
			tmpneur->threshold = perturb(tmpneur->threshold);
			tmpneur->threshold *= MAXTHRES;
		    }
		    if (sto.IRandomX(0,100) < PROBAMUT)
		    {
			switch (sto.IRandomX(0,1))
			{
			    case 0: tmpneur->function = TANH; break;
			    case 1: tmpneur->function = SIGMOID; break;
			}
		    }
		    /*if (random() % 100 < PROBAMUT)
		    {
			switch (random() % 3)
			{
			    case 0: tmpneur->function = INVEXP; break;
			    case 1: tmpneur->function = SIGMOID; break;
			    case 2: tmpneur->function = TANH; break;
			}
		    }*/
		    for (j=0; j < MAXCONFROM; j++)
		    {
			if (sto.IRandomX(0,100) < PROBAMUT )// / 2)
			{
			    if (tmpneur->confrom[j].exists == 0)
			    {
				tmpneur->confrom[j].exists = 1;
				randConn(num, i, j);
			    }
			    else
				tmpneur->confrom[j].exists = 0;
			}
			if (tmpneur->confrom[j].exists)
			{
			    if (sto.IRandomX(0,100) < PROBAMUT)
				randConn(num, i, j);
			    if (sto.IRandomX(0,100) < PROBAMUT)
				tmpneur->confrom[j].val =
				    perturb(tmpneur->confrom[j].val);
			    if (sto.IRandomX(0,100) < PROBAMUT)
			    {
				int tmp;
				tmp = sto.IRandomX(0,2);
				if (tmp == 0)
				    tmpneur->confrom[j].reftype = REFBOTH;
				else if (tmp == 1)
				    tmpneur->confrom[j].reftype = REFORIG;
				else
				    tmpneur->confrom[j].reftype = REFSYMM;
			    }
			    if (sto.IRandomX(0,100) < PROBAMUT)
			    {
				int tmp;
				tmp = sto.IRandomX(0,2);
				if (tmp == 0)
				    tmpneur->confrom[j].rectype = RECSELF;
				else if (tmp == 1)
				    tmpneur->confrom[j].rectype = RECDAD;
				else
				    tmpneur->confrom[j].rectype = RECSON;
			    }
			}

		    }

		}
	    }
	    // morphological mutation
	    for (i=0; i < 12; i++)
	    {
		if (sto.IRandomX(0,100) < PROBAMUT)
		    switch (i)
		    {
			case 0:
			    if (num != 0) genome[num].dadnum = sto.IRandomX(0,num-1);
			    break;
			case 1:
			    myprintf("old lgt %f",genome[num].lgt);
			    genome[num].lgt =
				perturbPositive(genome[num].lgt);
			    myprintf(", new lgt %f\n",genome[num].lgt);
			    break;
			case 2:
			    myprintf("old wdt %f",genome[num].lgt);
			    genome[num].wdt =
				perturbPositive(genome[num].wdt);
			    myprintf(", new wdt %f\n",genome[num].lgt);
			    break;
			case 3:
			    genome[num].hgt =
				perturbPositive(genome[num].hgt);
			    break;
			case 4:
			    myprintf("Old alpha: %f ", genome[num].alpha);
			    genome[num].alpha *= 8;
			    genome[num].alpha += (sto.IRandomX(0,4) - 2);
			    if (genome[num].alpha > 3)
				genome[num].alpha -= 8;
			    if (genome[num].alpha < -4)
				genome[num].alpha += 8;
			    genome[num].alpha = genome[num].alpha / 8.0;
			    //genome[num].alpha =
				//perturb(genome[num].alpha);
			    myprintf("New alpha: %f\n", genome[num].alpha);
			    break;
			case 5:
			    myprintf("Old beta: %f ", genome[num].beta);
			    genome[num].beta *= 8;
			    genome[num].beta += (sto.IRandomX(0,4) - 2);
			    if (genome[num].beta > 3)
				genome[num].beta -= 8;
			    if (genome[num].beta < -4)
				genome[num].beta += 8;
			    genome[num].beta = genome[num].beta / 8.0;
				//perturb(genome[num].beta);
			    myprintf("New beta: %f\n", genome[num].beta);
			    break;
			case 6:
			    if (sto.IRandomX(0,1))
				genome[num].orient = 1;
			    else genome[num].orient = -1;
			    break;
			case 7:
			    if (genome[num].ref1)
				genome[num].ref1 = 0;
			    else
			    {
				genome[num].ref1 = 1;
				// if we are symmetrising a limb, but this limb is in the
				// vertical plane, we would want to "angle" it a bit so the
				// two symmetric copies are not "melted" together
				if (genome[num].alpha == 0)
				{
				    if (sto.IRandomX(0,1))
					genome[num].alpha--;
				    else
					genome[num].alpha++;
				}
				if (genome[num].alpha == -4)
				{
				    if (sto.IRandomX(0,1))
					genome[num].alpha++;
				    else
					genome[num].alpha = 3;
				}
				if ((genome[num].beta == 2) || (genome[num].beta == -2))
				{
				    if (sto.IRandomX(0,1))
					genome[num].beta--;
				    else
					genome[num].beta++;
				}
			    }
			    break;
			case 8:
			    if (genome[num].recur)
				genome[num].recur = 0;
			    else
				if (num != 0)
				    genome[num].recur = sto.IRandomX(1,MAXRECUR-1);
			    break;
			case 9:
			    genome[num].terminal = 1-genome[num].terminal;
			    break;
			case 10:
			    myprintf("Old alpharec: %f ", genome[num].alpharec);
			    genome[num].alpharec = (dReal)(sto.IRandomX(0,7) - 4) / 8.0;
			    //genome[num].alpharec =
				//perturb(genome[num].alpharec);
			    myprintf("New alpharec: %f\n", genome[num].alpharec);
			    break;
			case 11:
			    myprintf("Old betarec: %f ", genome[num].beta);
			    genome[num].betarec = (dReal)(sto.IRandomX(0,7) - 4) / 8.0;
			    //genome[num].betarec =
				//perturb(genome[num].betarec);
			    myprintf("New betarec: %f\n", genome[num].betarec);
			    break;
		    }
	    }
	}
    }

void Animat::findClosestAnimat()
    {
	// find other animat currently alive whoe trunk (= root limb = limb 0)
	// is closest to my trunk
	dReal bestdist = 999999, dist;
	dReal *posother;
	dVector3 vectother;
	for (int n=0; n < REGISTSIZE; n++)
	{
	    if (!regist[n]) continue;
	    if (regist[n] == this) continue;
	    posother= (dReal*)
		dBodyGetPosition(regist[n]->limbs[0].id);
	    dBodyGetPosRelPoint(limbs[0].id, posother[0],
		    posother[1], posother[2], vectother);
	    dist = vectother[0]*vectother[0]
		+ vectother[1]*vectother[1]
		+ vectother[2]*vectother[2];
	    if (dist < bestdist)
	    {
		bestdist = dist;
		closestanimat = regist[n];
	    }
	}
    }

void Animat::actuate()
    {
	// neural actuation cycle: get sensor values, then update all neuron's
	// states and outputs
	int n, i, j, run; dReal speed, value;
	Neuron *tmpneur;
	speed=0;
	// get sensor data...
	fillSensors();
	//fillSensorsDist();
	for (run=0; run < NBNEURCYCLES; run++)
	{
	    for (n=0; n < MAXLIMBS; n++)
	    {
		if (repres[n].dead) continue;
		for (i=0; i < MAXNEUR; i++)
		{
		    tmpneur = &repres[n].neurons[i];
		    if (!tmpneur->exists) continue;
		    // don't touch input neurons !
		    if ((tmpneur->type == SENSPROP)
			    || (tmpneur->isExtSensor()))
			continue;
		    tmpneur->state = 0;
		    for (j=0; j < MAXCONFROM; j++)
			if (tmpneur->confrom[j].exists)
			{
			    if (repres[tmpneur->confrom[j].limb].dead)
			    {
				//mydie("Error: connection from dead limb\n");
                                myprintf("Error: connection from dead limb\n");
                                        return;
			    }
			    if (repres[tmpneur->confrom[j].limb].
				    neurons[tmpneur->confrom[j].neur].exists)
			    {
				value = 0;
				// If I'm ref'ed and conn is from my phys dad..
				// AND if i'm not in a recursion...
				if (genome[ repres[n].origgene ].ref1
					&& (tmpneur->confrom[j].limb
					    == repres[n].dadnum)
					&& (repres[n].origgene !=
				    repres[tmpneur->confrom[j].limb].origgene))
				{
				    if (tmpneur->confrom[j].reftype==REFBOTH)
					// both neurons can receive dad's
					// input
				value =
				    tmpneur->confrom[j].val *
				    repres[tmpneur->confrom[j].limb].
				    neurons[tmpneur->confrom[j].neur].out;
				    else
				    {
					int mysym;
					for (mysym=0; mysym< MAXLIMBS; mysym++)
					    if ((repres[mysym].origgene ==
						    repres[n].origgene) &&
						(repres[mysym].dadnum ==
						 repres[n].dadnum) &&
						(n != mysym) &&
						!repres[mysym].dead)
						break;
					if (mysym == MAXLIMBS)
					{
					    //mydie("Error: couldn't find symmetric of limb %d\n!", n);
                                            myprintf("Error 2: couldn't find symmetric of limb %d\n!",n);
                                        return;
					}
					if (((n < mysym) && (tmpneur->confrom[j].reftype == REFORIG))
					    || ((n > mysym) && (tmpneur->confrom[j].reftype == REFSYMM)))
					    value =
						tmpneur->confrom[j].val *
						repres[tmpneur->confrom[j].limb].
						neurons[tmpneur->confrom[j].neur].out;
					else
					    value = 0;

				    }

				}
				else if ((repres[tmpneur->confrom[j].limb].dadnum == n)
					&& genome[ repres[tmpneur->confrom[j].limb].origgene ].ref1
					&& (repres[tmpneur->confrom[j].limb].origgene
					  != repres[n].origgene))
				    // if conn is from my son and this son is
				    // reflected (i.e. duplicated through
				    // bilateral symmetry)..
				    // AND this is the original son (not the
				    // reflected/symmetrical version)...
				    // then I must find out how to deal with
				    // the input (i.e. take it from the
				    // original son, the reflected/symmetrical
				    // son, or averaging the 2)
				{
				    int zesym, zeson, tmp;
				    zeson = tmpneur->confrom[j].limb;
				    // finding the reflected copy of this son
				    for (zesym=0; zesym < MAXLIMBS; zesym++)
					if ((repres[zesym].origgene ==
						    repres[zeson].origgene) &&
						(repres[zesym].dadnum ==
						 repres[zeson].dadnum) &&
						(zeson != zesym) &&
						!repres[zesym].dead)
					    break;
				    if (zesym == MAXLIMBS)
				    {
					//mydie("Error 2: couldn't find symmetric of limb %d\n!",
					//	zeson);
                                        myprintf("Error 2: couldn't find symmetric of limb %d\n!",zeson);
                                        return;
				    }
				    // the lowest number is the original, the
				    // higher number is
				    // the symm
				    if (zesym < zeson) { tmp=zeson; zeson=zesym; zesym=tmp; }
				    if (tmpneur->confrom[j].reftype == REFORIG)
					value =
					    tmpneur->confrom[j].val *
					    repres[zeson].neurons[tmpneur->confrom[j].neur].out;
				    else if (tmpneur->confrom[j].reftype == REFSYMM)
					value =
					    tmpneur->confrom[j].val *
					    repres[zesym].neurons[tmpneur->confrom[j].neur].out;
				    else if (tmpneur->confrom[j].reftype == REFBOTH)
					value =
					    tmpneur->confrom[j].val *
					    (repres[zesym].neurons[tmpneur->confrom[j].neur].out
					     +repres[zeson].neurons[tmpneur->confrom[j].neur].out)
					    /2.0;
				}
				else
				{
				    value =
					tmpneur->confrom[j].val *
					repres[tmpneur->confrom[j].limb].
					neurons[tmpneur->confrom[j].neur].out;
			    /*myprintf("%.4f l%d n%d c%d from l%d n%d\n",
					    value, n, i, j,
					    tmpneur->confrom[j].limb,
					    tmpneur->confrom[j].neur
					    );*/
				}
				tmpneur->state += value;
				/*if ((n == 1) && (i == 1) && (j == 0))
				    myprintf("neur 1:1 receives %.3f from conn 0\n", value);*/
			    }
			}
		}
	    }

	    // neuron's internal states have been updated from inputs. Now
	    // calculating outputs from those states...
	    for (n=0; n < MAXLIMBS; n++)
	    {
		if (repres[n].dead) continue;
		for (i=0; i < MAXNEUR; i++)
		{
		    tmpneur = &repres[n].neurons[i];
		    // again, don't touch sensor neurons (their output is
		    // already filled by fillSensors()
		if ((tmpneur->type == SENSPROP)
			    || (tmpneur->isExtSensor()))
		    continue;
		    if (tmpneur->exists)
		    {
			int c, nbconns;
			nbconns = 0;
			for (c=0; c < MAXCONFROM; c++)
			    if (tmpneur->confrom[c].exists
				    && !repres[tmpneur->confrom[c].limb].dead // shouldn't happen
				    && repres[tmpneur->confrom[c].limb].neurons[tmpneur->confrom[c].neur].exists)
				nbconns++;


			if ((nbconns == 0) && (tmpneur->state != 0))
			{
			    mydie("Damn ! receives input without conns ! State:%f\n",
				    tmpneur->state);
			}


			if (nbconns == 0)
			    tmpneur->state = 0;
			else
			    tmpneur->state = tmpneur->state / (dReal)nbconns;


			if (tmpneur->type == ACTUATOR)
			{
			    //myprintf("%.3f\n", tmpneur->state);
			    tmpneur->out =
				mytanh(tmpneur->state);
				//mytanh(tmpneur->state + tmpneur->threshold);
				//tmpneur->state + tmpneur->threshold;
			}
			else
			{
			    if (tmpneur->function == SIGMOID)
				tmpneur->out =
				    sigmoid(tmpneur->state + tmpneur->threshold);
			    /*else if (tmpneur->function == INVEXP)
				tmpneur->out =
				    invexp(tmpneur->state + tmpneur->threshold);*/
			    else if (tmpneur->function == TANH)
				tmpneur->out =
				    mytanh(tmpneur->state + tmpneur->threshold);
			    else
				mydie("Error : Unknown output function!\n");
			}

		    }
		    //   tmpneur->out = 1;
		    else
			tmpneur->out = 0;


		}
	    }
	}

	/*myprintf("Neur 1:3 has out %.4f\n", repres[1].neurons[3].out);
	myprintf("Neur 2:3 has out %.4f\n", repres[2].neurons[3].out);
	myprintf("Neur 0:3 has state %.4f\n", repres[0].neurons[3].state);*/

	for (i=1; i < nblimbs(); i++)
	{
            if (NOACTUATE)
	    {
		dJointSetHingeParam(limbs[i].joint, dParamFMax, 0);
	    }
	    else
	    {
		dJointSetHingeParam(limbs[i].joint, dParamFMax, MAXFORCE*10);
		if (FREEZE) speed = 0;
		else
		{
		    // damping...
		    speed = 2.0 * limbs[i].oldspeed + SPEEDMULT *
			repres[i].neurons[0].out ;
		    speed = speed / 3.0;
		    limbs[i].oldspeed = speed;
		}
		dJointSetHingeParam(limbs[i].joint, dParamVel, speed);

		/*if (VISUAL)
		    myprintf("Limb %d : Act %.3f, Sen %f, Angle %f, Rate %f\n",
			    i, repres[i].neurons[0].out,
			    repres[i].neurons[2].out,
			    dJointGetUniversalAngle1(limbs[i].joint),
			    0.0);
		//dJointGetUniversalAngleRate1 (limbs[i].joint));*/
	    }
	}
    }

void Animat::makeNeuronsLimb(int limb)
    {
	// generates random, valid neural information for this limb (i.e. gives
	// an actuator and a proprioceptor, etc.)
	int neur;
	//myprintf("Making neurs for limb %d\n", limb);
	for (neur=0; neur < MAXNEUR; neur++)
	{
	    genome[limb].neurons[neur].exists = 0;

	    if ((neur == 0) && (limb != 0))
	    {
		genome[limb].neurons[neur].type = ACTUATOR;
		genome[limb].neurons[neur].exists = 1;
		genome[limb].neurons[neur].randVals();
	    }
	    //else if (((neur == 3) && (limb == 0)) ||
	    else if ((neur == 2) && (limb != 0))
	    {
		genome[limb].neurons[neur].type = SENSPROP;
		genome[limb].neurons[neur].exists = 1;
		genome[limb].neurons[neur].randVals();
	    }
	    else if (sto.IRandomX(0,100) < PROBANEURSENS)
	    {
		int val = sto.IRandomX(0, NBSENSORTYPES);
		genome[limb].neurons[neur].type = SENSORTYPES[val];
		genome[limb].neurons[neur].exists = 1;
		genome[limb].neurons[neur].randVals();
	    }
	    else
	    {
		genome[limb].neurons[neur].type = INTER;
		if (sto.IRandomX(0, 100) < PROBANEUREXISTS)
		{
		    genome[limb].neurons[neur].exists = 1;
		    genome[limb].neurons[neur].randVals();
		}
	    }
	}
    }

void Animat::randConnectionsLimb(int limb)
    {
	// randomise neural connections within this limb
	int neur;
	if (genome[limb].dead) return;
	for (neur=0; neur < MAXNEUR; neur++)
	{
	    if ( ! genome[limb].neurons[neur].exists) continue;
	    randConnectionsNeur(limb, neur);
	}
    }

void Animat::randConnectionsNeur(int limb, int neur)
    {
	// randomise neural connections within this neuron
	int conn;
	for (conn=0; conn < MAXCONFROM; conn++)
	{
	    if (sto.IRandomX(0 , 100) > PROBACONNEXISTS)
	    {
		genome[limb].neurons[neur].confrom[conn].exists = 0;
	    }
	    else
	    {
		genome[limb].neurons[neur].confrom[conn].exists = 1;
		randConn(limb, neur, conn);
	    }
	}
    }

void Animat::printRepres()
    {
	int i;
	for (i=0; i < MAXLIMBS; i++)
	    myprintf("%d", repres[i].dead);
	myprintf("\n");
    }

void Animat::printGenome()
    {
	int i;
	for (i=0; i < MAXGENES; i++)
	    myprintf("%d", genome[i].dead);
	myprintf("\n");
    }

void Animat::printGenomeFull()
    {
	int i;
	for (i=0; i < MAXGENES; i++)
	{
	    if (genome[i].dead)
		myprintf("1 ");
	    else
		myprintf("0 (%f, %f, %f) ",
			genome[i].lgt, genome[i].wdt, genome[i].hgt);
	}
	myprintf("\n");
    }

void Animat::randGenome()
    {
	// creates a random, valid genome. Does NOT test for internal
	// collisions

	int idx, nbg;
	int i,  mydad;
	//int j, nbsons;
	genome[0].dead = 0;
	for (i=1; i < MAXGENES; i++) genome[i].dead = 1;
	genome[0].randBodyGene(-1,Limb::TORSO);
        nbg =  sto.IRandomX(MINGENES,MAXGENES);
	for (idx=1; idx < nbg-1; idx++)
	{
	    // Only choose a dad if it has max. 2 other sons
	    //  => 4 links max (3 sons, 1 dad)
	    /*do
	    {*/
		mydad = sto.IRandomX(0,idx-1);
		/*nbsons = 0;
		for (j=0; j < idx; j++)
		    if (genome[j].dadnum == mydad)
			nbsons ++;
	    } while (nbsons > 2);*/
	    genome[idx].randBodyGene(mydad);
	    genome[idx].dead = 0;
	}
             
        mydad=sto.IRandomX(1,nbg-2);
        int dadad=mydad;
        while(dadad>-1)
        {
            genome[dadad].ref1=0;
            genome[dadad].recur=0;
            dadad = genome[dadad].dadnum;
        }
        
        genome[nbg-1].randBodyGene(mydad,Limb::HEAD);
        genome[nbg-1].dead=0;
	if (idx < MINGENES-1) {mydie("Damn! Not enough genes!"); }
	for (i=0; i < MAXGENES; i++)
	   makeNeuronsLimb(i);
	for (i=0; i < MAXGENES; i++)
	   randConnectionsLimb(i);
	checkGenome();
    }

    // Note: all "push.." functions must be called while the animat is alive
    // (i.e. after it's been generated) - obviously.

void Animat::pushBeforeXVert(dReal X)
    {
	// push the indiv on the x+ side of a horizontal line
	dReal *trunkpos;
	dReal aabb[6];

	trunkpos = (dReal* )dBodyGetPosition(limbs[0].id);
	dGeomGetAABB ((dGeomID)space, aabb);
	setPos(X + (trunkpos[0] - aabb[0]), trunkpos[1]);
    }

void Animat::pushBehindXVert(dReal X)
    {
	// push the indiv on the x- side of a horizontal line
        const dReal *trunkpos = dBodyGetPosition(limbs[0].id);
	dReal aabb[6];
        if(!limbs[0].id)myprintf("ERROR: NO LIMB ZERO.");
	dGeomGetAABB ((dGeomID)space, aabb);
	setPos(X - (aabb[1] - trunkpos[0]), trunkpos[1]);
    }

void Animat::pushBehindX(dReal X)
    {
	// push the indiv on the x- side of a slanted plane (slanted 45 deg.
	// towards negative x), as in Sims' "Coevolving artificial creatures"
	// paper

	int touches;
	dVector3 sides = {.1,20,20}; // vertical 'slice'
	dVector3 centre = {X, 0, 0};
	dMatrix3 rot;
	dVector3 limbsides;
	dReal *trunkpos;
	dReal xshift;
	dReal aabb[6];

	trunkpos = (dReal* )dBodyGetPosition(limbs[0].id);
	dGeomGetAABB ((dGeomID)space, aabb);
	xshift = aabb[5]-aabb[4] + aabb[1]-trunkpos[0]; // push by x+z
	setPos(X-xshift, trunkpos[1]);

	dRFromAxisAndAngle(rot, 0, 1, 0, -(M_PI / 4.0)); // slant back 45deg
	do
	{
	    touches = 0;
	    for (int i=0; i < MAXLIMBS; i++)
		if (limbs[i].id)
		{
		    dGeomBoxGetLengths(limbs[i].geom, limbsides);
		    touches = dBoxTouchesBox(centre, rot, sides,
			    dBodyGetPosition(limbs[i].id),
			    dBodyGetRotation(limbs[i].id),
			    limbsides
			    );
		    if (touches) break;
		}
	    if (!touches)
	    {
		setPos(dBodyGetPosition(limbs[0].id)[0] + 0.1,
			dBodyGetPosition(limbs[0].id)[1]);
		myprintf("not touching!\n");
	    }
	} while (!touches);
	myprintf("touching!\n");
    }

void Animat::pushBeforeX(dReal X)
    {
	// push the indiv on the x+ side of a slanted plane (slanted 45 deg.
	// towards positive x), as in Sims' "Coevolving artificial creatures"
	// paper
	int touches;
	dVector3 sides = {.1,20,20}; // vertical 'slice'
	dVector3 centre = {X, 0, 0};
	dMatrix3 rot;
	dVector3 limbsides;
	dReal *trunkpos;
	dReal xshift;
	dReal aabb[6];

	trunkpos = (dReal* )dBodyGetPosition(limbs[0].id);
	dGeomGetAABB ((dGeomID)space, aabb);
	xshift = aabb[5]-aabb[4] + trunkpos[0]-aabb[0]; // push by x+z
	setPos(X+xshift, trunkpos[1]);

	dRFromAxisAndAngle(rot, 0, 1, 0, (M_PI / 4.0)); // slant forth 45deg
	do
	{
	    touches = 0;
	    for (int i=0; i < MAXLIMBS; i++)
		if (limbs[i].id)
		{
		    dGeomBoxGetLengths(limbs[i].geom, limbsides);
		    touches = dBoxTouchesBox(centre, rot, sides,
			    dBodyGetPosition(limbs[i].id),
			    dBodyGetRotation(limbs[i].id),
			    limbsides
			    );
		    if (touches) break;
		}
	    if (!touches)
	    {
		setPos(dBodyGetPosition(limbs[0].id)[0] - 0.1,
			dBodyGetPosition(limbs[0].id)[1]);
		myprintf("not touching!\n");
	    }
	} while (!touches);
	myprintf("touching!\n");
    }

int Animat::isInSublimbGenome(int limb, int anc)
    {
	// to be used for the genome only
	// Is gene "anc" part of "limb"'s sub-tree (i.e. children limbs + their
	// own children, etc.) ?
	int num;
	//myprintf("In isInSubLimb\n");
	if (genome[limb].dead) return 0;
	//displayGenome();
	num = limb;
	if (num == anc) return 1;
	do {
	    //myprintf("going from %d to its dad %d, ",num, genome[num].dadnum);
	    if (num == genome[num].dadnum)
	    {
		myprintf("Damn, %d has dad %d !\n", num, genome[num].dadnum);
		exit(0);
	    }
	    num = genome[num].dadnum;
	} while ((num != -1) && (num != anc));
	//if (num == anc) myprintf(" %d == ancestor sought %d\n", num, anc);
	//	 else myprintf("%d is not part of %d sublimb\n", limb, anc);
	if (num == anc) return 1; else return 0;

    }

int Animat::isInSublimbRepres(int limb, int anc)
    {
	// to be used for the repres, that is, the developed genome that
	// contains all limbs (including duplicates, symmetrics, etc.)
	// Is limb "anc" part of "limb"'s sub-tree (i.e. children limbs + their
	// own children, etc.) ?
	int num;
	if (repres[limb].dead) return 0;
	num = limb;
	if (num == anc) return 1;
	do {
	    num = repres[num].dadnum;
	} while ((num != -1) && (num != anc));
	if (num == anc) return 1; else return 0;

    }

void Animat::transGenome()
    {
	// Reads the genome into the repres, taking into account the
	// developmental information (recursivity, symmetry, etc.)
	int i;
	/*transGenome(0, -1, 1, 1, 0);
	myprintf("\n");*/
	for (i=0; i < MAXLIMBS; i++)
	{
	    repres[i].dead=1;
	    repres[i].symmetrised = 0;
	}
	transGeneSeg(0, -1, 0, 0);
	transNeurons();
    }

void Animat::transGeneSeg(int zegene, int zedad, int numrec, int symm)
    {
	// Does the real work of the previous function, but does not correctly
	// update neurons (transNeurons() does that) .

	// Note: This is a recursive function (as suits the tree structure of
	// genomes)

	int limbnum, i;
	//myprintf("transGeneSeg(%d, %d, %d, %d)\n",
	//	zegene, zedad, numrec, symm);
	limbnum = nblimbs();
	if (limbnum >= MAXLIMBS) return;
	repres[limbnum] = genome[zegene];
	repres[limbnum].dead = 0;
	repres[limbnum].origgene = zegene;
	repres[limbnum].dadnum = zedad;
        repres[limbnum].type = genome[zegene].type;

	if (symm)
	    repres[limbnum].symmetrised = 1-repres[zedad].symmetrised;
	else
	    repres[limbnum].symmetrised = repres[zedad].symmetrised;
	if (repres[limbnum].symmetrised)
	    repres[limbnum].alpha *= -1;
	if (repres[zedad].origgene == zegene) // recursion !
	{
	    repres[limbnum].alpha = repres[limbnum].alpharec / 4.0;
	    repres[limbnum].beta = repres[limbnum].betarec / 4.0;
	    repres[limbnum].alpha = 0;
	    repres[limbnum].beta = 0;
	    if (repres[limbnum].symmetrised)
		repres[limbnum].alpha *= -1;
	}
	for (i=0; i < MAXGENES; i++)
	    if ((genome[i].dadnum == zegene) && !genome[i].terminal
		    && !genome[i].dead)
	    {
		transGeneSeg(i, limbnum,  genome[i].recur, 0);
		if (genome[i].ref1)
		    transGeneSeg(i, limbnum,  genome[i].recur, 1);
	    }
	if (numrec && (limbnum != 0)) // LIMB 0 CANNOT BE RECURRED (control centre)
	{
	    //myprintf("Recur in gene %d\n", zegene);
	    transGeneSeg(zegene, limbnum, numrec -1, 0);
	    limbnum = nblimbs()-1;
	}
	for (i=0; i < MAXGENES; i++)
	    if ((genome[i].dadnum == zegene) && genome[i].terminal
		    && !genome[i].dead && !numrec)
	    {
		transGeneSeg(i, limbnum,  genome[i].recur, 0);
		if (genome[i].ref1)
		    transGeneSeg(i, limbnum,  genome[i].recur, 1);
	    }

    }

void Animat::transNeurons()
    {
	// develops the neural information from the genome into the repres,
	// taking into account developmental info (i.e. recursive segmentation,
	// symmety, etc.)

	// Tricky code.

	int limbnum, neur, conn, i, found;
	Neuron *tmpneur;
	for (limbnum=0; limbnum < MAXLIMBS; limbnum++)
	{
	    if (repres[limbnum].dead) break;
	    for (neur=0; neur < MAXNEUR; neur++)
	    {
		tmpneur = &repres[limbnum].neurons[neur];
		if (!tmpneur->exists) continue;
		for(conn=0; conn < MAXCONFROM; conn++)
		{
		    found = 0;
		    if (!tmpneur->confrom[conn].exists) continue;
		    if (tmpneur->confrom[conn].limb == repres[limbnum].origgene)
		    {
			// if conn comes from my own origgene.. then we must
			// check
			// whether I'm being recursively duplicated, and if so,
			// check the genetic info to know how to deal with the
			// connection
			int isinrec = 0;
			if (repres[limbnum].dadnum != -1)
			{
			    if (repres[limbnum].origgene ==
				repres[repres[limbnum].dadnum].origgene)
			    {
				isinrec = 1;
				if (tmpneur->confrom[conn].rectype == RECDAD)
				{
				    tmpneur->confrom[conn].limb=
					repres[limbnum].dadnum;
				    found = 1;
				}
			    }
			}
			for (i=0; i < MAXLIMBS; i++)
			{
			    if (repres[i].dead) break;
			    if ((repres[i].dadnum == limbnum) &&
			        (repres[i].origgene==repres[limbnum].origgene))
			    {
				isinrec = 1;
				if (tmpneur->confrom[conn].rectype ==
					RECSON)
				{
				    tmpneur->confrom[conn].limb = i;
				    found = 1;
				}
			    }
			}
			if (isinrec && (!found))
			{
			    if (tmpneur->confrom[conn].rectype == RECSELF)
				tmpneur->confrom[conn].limb = limbnum;
			    else
				tmpneur->confrom[conn].exists = 0;
			    found = 1;
			}
			else if (!found)
			{
			    tmpneur->confrom[conn].limb = limbnum;
			    found = 1;
			}
		    }
		    else
		    {
			if ((tmpneur->confrom[conn].limb == 0) &&
			     (repres[limbnum].origgene !=
				    repres[repres[limbnum].dadnum].origgene))
			{
			    found = 1;
			}
			if (limbnum == 0)
			{
			    for (i=0; i < MAXLIMBS; i++)
			    {
				if (repres[i].dead) break;
				if (repres[i].origgene != repres[repres[i].dadnum].origgene)
				    if (tmpneur->confrom[conn].limb ==
					    repres[i].origgene)
				    {
					tmpneur->confrom[conn].limb = i;
					found = 1;
				    }
			    }
			}
			if (repres[limbnum].dadnum != -1)
			{
			    if (tmpneur->confrom[conn].limb ==
					repres[repres[limbnum].dadnum].origgene)
			    {
				tmpneur->confrom[conn].limb =
				    repres[limbnum].dadnum;
				found = 1;
			    }
			}
			for (i=0; i < MAXLIMBS; i++)
			{
			    if (repres[i].dead) break;
			    if (repres[i].dadnum == limbnum)
				if (tmpneur->confrom[conn].limb ==
					repres[i].origgene)
				{
				    tmpneur->confrom[conn].limb = i;
				    found = 1;
				}
			}
		    }
		    if (!found)
		    {
			tmpneur->confrom[conn].exists = 0;
			/*myprintf("Warning: Could not transcribe conn %d:%d in limb %d\n",
				neur, conn, limbnum);*/
		    }
		    if (tmpneur->confrom[conn].limb == -1)
		    {
			mydie("Conn from -1 !\n");
		    }

		}
	    }
	}

    }

void Animat::dev1RecInGenome()
    {
	// finds a gene that carries a recurrence flag (recursive duplication,
	// i.e. segmentation) and turns it into several independent (though
	// identical) genes, which will now be able to evolve independently.
	int i, j, recs[MAXGENES], nbrecs;
	nbrecs=0;
	for (i=1; i < MAXGENES; i++) // YOU CANNOT RECUR LIMB 0
	    if ((!genome[i].dead) && (genome[i].recur > 0))
	    {
		recs[nbrecs] = i;
		nbrecs++;
	    }
	if (nbrecs)
	{
	    i = sto.IRandomX(0, nbrecs-1);
	    Animat tmpA;
	    myprintf("\nrec found at gene %d !\n", recs[i]);
	    tmpA.copyFrom(this);
	    for (j=0; j < MAXLIMBS; j++)
	    {
		tmpA.repres[j].dead=1;
		tmpA.repres[j].symmetrised = 0;
	    }
	    tmpA.transGeneDev1Rec(0, -1, 0, 0, recs[i]);
	    tmpA.transNeurons();
	    myprintf("\nDeveloped genome has %d limbs\n", tmpA.nblimbs());
	    myprintf("\nBefore development:\n", tmpA.nblimbs());
	    displayGenomeFull();
	    if (tmpA.nblimbs() < MAXGENES)
		for (j=0; j < MAXGENES; j++)
		    genome[j] = tmpA.repres[j];
	    else myprintf("No copying, too big\n");
	    myprintf("\nAfter development:\n", tmpA.nblimbs());
	    displayGenomeFull();
	}
	else myprintf("\nNo rec found to develop !\n");
    }

void Animat::transGeneDev1Rec(int zegene, int zedad, int numrec, int symm, int genetodev)
    {
	// Used in the previous function.

	int limbnum, i;
	//myprintf("transGeneSeg(%d, %d, %d, %d)\n",
	//	zegene, zedad, numrec, symm);
	limbnum = nblimbs();
	if (limbnum >= MAXLIMBS) return;
	repres[limbnum] = genome[zegene];
	repres[limbnum].dead = 0;
	repres[limbnum].origgene = zegene;
	repres[limbnum].dadnum = zedad;
	if (symm)
	    repres[limbnum].symmetrised = 1-repres[zedad].symmetrised;
	else
	    repres[limbnum].symmetrised = repres[zedad].symmetrised;
	if (repres[limbnum].symmetrised)
	    repres[limbnum].alpha *= -1;
	if (repres[zedad].origgene == zegene) // recursion !
	{
	    repres[limbnum].recur = 0;
	    repres[zedad].recur= 0;
	    //repres[limbnum].alpha = repres[limbnum].alpharec / 4.0;
	    //repres[limbnum].beta = repres[limbnum].betarec / 4.0;
	    repres[limbnum].alpha = 0;
	    repres[limbnum].beta = 0;
	    if (repres[limbnum].symmetrised)
		repres[limbnum].alpha *= -1;
	}
	for (i=0; i < MAXGENES; i++)
	    if ((genome[i].dadnum == zegene) && !genome[i].terminal
		    && !genome[i].dead)
	    {
		transGeneDev1Rec(i, limbnum,  genome[i].recur, 0, genetodev);
		//if (genome[i].ref1)
		//    transGeneDev1Rec(i, limbnum,  genome[i].recur, 1, genetodev);
	    }
	if (numrec && (zegene == genetodev)) // LIMB 0 CANNOT BE RECURRED (control centre)
	{
	    //myprintf("Recur in gene %d\n", zegene);
	    transGeneDev1Rec(zegene, limbnum, numrec -1, 0, genetodev);
	    limbnum = nblimbs()-1;
	}
	for (i=0; i < MAXGENES; i++)
	    if ((genome[i].dadnum == zegene) && genome[i].terminal
		    && !genome[i].dead && !numrec)
	    {
		transGeneDev1Rec(i, limbnum,  genome[i].recur, 0, genetodev);
		//if (genome[i].ref1)
		//    transGeneDev1Rec(i, limbnum,  genome[i].recur, 1, genetodev);
	    }

    }

    /*void transGenome(int g, int dad,int malpha, int mbeta, int ignoresym)
    {
	int i, j, num;
	static int r;
	// initialisation occurs if specified dad is -1
	if (dad == -1)
	{
	    for (i=0; i < MAXLIMBS; i++)
		repres[i].dead = 1;
	    r = 0;
	}
	//printf ("limb %d, ", r);
	if (r >= MAXLIMBS) return;
	repres[r] = genome[g];

	if (malpha == -1) repres[r].symmetrised = 1;
	    else repres[r].symmetrised = 0;

	repres[r].dadnum = dad;
	repres[r].alpha *= malpha;
	repres[r].beta *= mbeta;

	for (i=0; i < MAXNEUR; i++)
	    for (j=0; j < MAXCONFROM; j++)
		if (repres[r].neurons[i].confrom[j].limb == genome[g].dadnum)
		    repres[r].neurons[i].confrom[j].limb = dad;
	num = r;
	r++;
	if (r >= MAXLIMBS) return;
	for (i=0; i < MAXGENES; i++)
	    if ((genome[i].dadnum == g) && (genome[i].dead == 0))
		transGenome(i, num, malpha, mbeta, 0);
	if (!ignoresym)
	{
	    if (genome[g].ref1)
	    {
		transGenome(g, dad, -malpha, mbeta, 1);
	    }
	}
    }*/

void Animat::setPos(dReal nx, dReal ny)
    {
	// set position in the x,y plane
	dReal *trunkpos;
	dReal aabb[6];
	dGeomGetAABB ((dGeomID)space, aabb);
	trunkpos = (dReal *)dBodyGetPosition(limbs[0].id);
	setPos (nx, ny, (trunkpos[2]-aabb[4]) + 0.01);
    }

void Animat::setPos(dReal nx, dReal ny, dReal nz)
    {
	// set position in space
	int i;
	dReal ox, oy, oz;
	dReal *tmpvect, *trunkpos;;
	trunkpos = (dReal *)dBodyGetPosition(limbs[0].id);
	ox = trunkpos[0];
	oy = trunkpos[1];
	oz = trunkpos[2];
	for (i=0; i < MAXLIMBS; i++)
	    if (limbs[i].id)
	    {
		tmpvect = (dReal *)dBodyGetPosition(limbs[i].id);
		dBodySetPosition (limbs[i].id, nx + (tmpvect[0]-ox),
			    ny + (tmpvect[1]-oy), nz + (tmpvect[2]-oz));
	    }
    }

void Animat::rotateByQ(dQuaternion q)
    {
	// rotate me by Quaternion q's angle
	//
	// get Pos of each limb, in limb 0's frame of ref
	// rotate limb 0
	// for each limb:
	// convert saved relative Pos to global coords, apply
	// rotate body of limb
	dVector3 oldpos[MAXLIMBS];
	dQuaternion qtmp;
	dVector3 postmp;
	for (int i=0; i < MAXLIMBS; i++)
	    if (limbs[i].id)
		dBodyGetPosRelPoint(limbs[0].id,
			dBodyGetPosition(limbs[i].id)[0],
			dBodyGetPosition(limbs[i].id)[1],
			dBodyGetPosition(limbs[i].id)[2],
			oldpos[i]);
	for (int i=0; i < MAXLIMBS; i++)
	    if (limbs[i].id)
	    {
		// qtmp = q * body's current quaternion
		dQMultiply0(qtmp, q, dBodyGetQuaternion(limbs[i].id));
		dBodySetQuaternion(limbs[i].id, qtmp);
	    }
	for (int i=1; i < MAXLIMBS; i++)
	    if (limbs[i].id)
	    {
		dBodyGetRelPointPos(limbs[0].id,
			oldpos[i][0],
			oldpos[i][1],
			oldpos[i][2],
			postmp);
		dBodySetPosition(limbs[i].id,
			postmp[0], postmp[1], postmp[2]);
	    }
    }

void Animat::dropRandomOnSphere()
    {
	// drop me at random over the sphere
	//
	//Of course, I must be alive (i.e. generated), and the world must be
	//spherical  !
	//
	// How it works:
	// 1 - pick a point at random in the 3D sphere centred at (0,0,0)
	// 2 - calculate the latitude and longitude of that point
	// 3 - drop on the sphere at that lati/longi
	// Why we need this: dropping at random lati/longi would result in
	// "crowded poles" (at smaller latitudes, a certain longitude diff
	// corresponds to a smaller distance)
	dReal x, y, z, dist, lati, longi;
	if (WORLDTYPE != SPHERICWORLD)
	{
	    myprintf("Trying to drop me on a flat world !\n");
	    mydie("Damn! Trying to drop animat on a flat world !\n");
	}
	do{
	    do{
		x = (dReal) (10000 - sto.IRandomX(0, 20000)) / 100.0;
		y = (dReal) (10000 - sto.IRandomX(0, 20000)) / 100.0;
		z = (dReal) (10000 - sto.IRandomX(0, 20000)) / 100.0;
	    }
	    while (sqrt(x*x + y*y + z*z) > 9999);

	    dist = sqrt(x*x + y*y + z*z);
	    lati = acos(z / dist); // z / R
	    // distance from the z axis
	    dist = sqrt(x*x + y*y);
	    longi = asin(x / dist);
	    if (y > 0) longi = M_PI - longi;
	    dropLatiLongi(lati, longi);
	    AIRCOLLISIONS = 0;
	    for (int i=0; i < REGISTSIZE; i++)
		if ((regist[i]) && (regist[i] != this))
		{
		    dSpaceCollide2((dxGeom*)(space),
			    (dxGeom*)(regist[i]->space), 0,
			    &airCallback);
		}
	    myprintf("Trying\n");
	} while (AIRCOLLISIONS);
	AIRCOLLISIONS = 0;
	dJointGroupEmpty (contactgroup);
    }

dReal Animat::getLongi()
    {
	// get my current longitude (in a spherical world)
	dReal *pos = (dReal *) dBodyGetPosition(limbs[0].id);
	dReal x = pos[0]; dReal y = pos[1];
	dReal dist = sqrt(x*x + y*y);
	dReal longi = asin (x / dist);
	if (y > 0)  longi = M_PI - longi;
	return longi;
    }

dReal Animat::getLati()
    {
	// get my current latitude (in a spherical world)
	dReal *pos = (dReal *) dBodyGetPosition(limbs[0].id);
	dReal x = pos[0]; dReal y = pos[1]; dReal z = pos[2];
	dReal dist = sqrt(x*x + y*y + z*z);
	return acos (z / dist);
    }

void Animat::dropLatiLongi(dReal lati, dReal longi)
    {
	// drop me at a certain latitude an longitude, on a spherical world
	//
	// NOTE: north pole = latitude 0, south pole = latitude M_PI
	// Longi: "Greenwich" (longitude 0) is the negative-y portion of the
	// intersec of the sphere with
	// the Oyz plane (i.e. it's the vertical half-meridian that lies
	// directly on the left of the sphere when seeing it from a point on
	// the positive half of the x axis).
	dVector3 v;
	setPos(0, 0);
	// First we're putting the creature in the right orientation
	dReal zoff = dBodyGetPosition(limbs[0].id)[2];
	dQuaternion q;
	dQFromAxisAndAngle(q, 1, 0, 0, lati);
	rotateByQ(q);
	dQFromAxisAndAngle(q, 0, 0, 1, longi);
	rotateByQ(q);
	// Now the position vector we want is simply the (0,0, RADIUS + zoff)
	// vector from limb 0, translated into global coords (because the
	// creature has the correct orientation).
	dBodyVectorToWorld(limbs[0].id, 0, 0, WORLDRADIUS + zoff + DROPHEIGHT, v);
	setPos(v[0], v[1], v[2]);
    }

int Animat::test_for_intra_coll()
    {
	// test whether my limbs are colliding with each other

	int a, b, res;
	res = 0;
	dSpaceCollide (globalspace,0,&nearCallback);
	for (a=0; a < nblimbs(); a++)
	{
	    for (b=0; b < nblimbs(); b++)
	    {
		// If two bodies are connected by anything else than a
		// Hinge, it is a non-allowed contact, because the
		// collision callback does not add contact joints between
		// hinge-connected limbs.
		if (dAreConnectedExcluding(limbs[a].id, limbs[b].id,
			    dJointTypeHinge))
		{
		    //myprintf("IntraColl: %d, %d !!\n",a, b);
		    /*int i, nbj; dJointID jointid;
		    nbj = dBodyGetNumJoints(limbs[a].id);
		    for (i=0; i < nbj; i++)
		    {
			jointid = dBodyGetJoint(limbs[a].id, i);
			if ((dJointGetBody(jointid, 0) == limbs[b].id)
			 || (dJointGetBody(jointid, 1) == limbs[b].id))
			{
			    printf("Joint between %d and %d of type %d (%d)\n",
				    a, b, dJointGetType(jointid),
			    dJointTypeContact);
			}
		    }*/
		    res = 1;
		}
	    }
	}
	dJointGroupEmpty (contactgroup);
	return res;
    }

void Animat::fillSensors()
{
    // update sensors information

    // note: if you're going to add / change available sensors, be sure to also
    // change SENSORTYPES and NBSENSORTYPES at the beginning of this file.

    int i, neur;
    for (i=0; i < nblimbs(); i++)
    {
        
	if (i != 0)
	    repres[i].neurons[2].out =
		dJointGetHingeAngle(limbs[i].joint)
		/ M_PI;

	for (neur=0; neur < MAXNEUR; neur++)
	{
	    if (!repres[i].neurons[neur].exists) continue;
            switch(repres[i].neurons[neur].type)
            {
                case ACCELX:
                repres[i].neurons[neur].out = tanh(limbs[i].acceleration[0]/5.0);
                break;

                case ACCELY:
                repres[i].neurons[neur].out = tanh(limbs[i].acceleration[1]/5.0);
                break;

                case ACCELZ:
                repres[i].neurons[neur].out = tanh(limbs[i].acceleration[2]/5.0);
                break;

                case ANGACCELX:
                repres[i].neurons[neur].out = tanh(limbs[i].angular_acceleration[0]/5.0);
                break;

                case ANGACCELY:
                repres[i].neurons[neur].out = tanh(limbs[i].angular_acceleration[1]/5.0);
                break;

                case ANGACCELZ:
                repres[i].neurons[neur].out = tanh(limbs[i].angular_acceleration[2]/5.0);
                break;

                default:
                    repres[i].neurons[neur].out =0;
            }
            if (DISABLESENSORS)
		    repres[i].neurons[neur].out = 0;
	}
    }
}

void Animat::remove()
{
    // remove me from the simulation

    int i;
    myprintf("Remove\n");
    if (!alive)
    {
	mydie("Damn ! Trying to remove a non-living animat !\n");
    }

    dSpaceDestroy(space);
    resetDamages();
    for (i=0; i < MAXLIMBS; i++)
    {
	if (limbs[i].id)
	{
	    if (limbs[i].joint)
	    {
		//printf ("%d\n ", (int)limbs[i].joint);
		dJointDestroy (limbs[i].joint);
		limbs[i].joint = 0;
	    }
	    dBodyDestroy(limbs[i].id);
	    limbs[i].id = 0;
            limbs[i].reset();
	}
    }
    i=0; while ((i < REGISTSIZE) && (regist[i] != this)) i++;
    if (i >= REGISTSIZE)
    {
	mydie("Damn ! Couldn't find animat in register while removing !\n");
    }
    regist[i] = NULL;
    alive = 0;
    nbLimbs=0;
    vect2d tmp(0,0);
    motionSum = tmp;
    while(!Motion.empty()){Motion.pop();Height.pop();}
}

void Animat::generate(dReal nx, dReal ny, dReal alpha)
{

    // put me into the simulation (make me "alive")

    // Assumes that the repres has been filled beforehand !
    int i, j;
    dReal aabb[6];
    dReal *trunkpos;
    myprintf("Generating..\n");
    if (alive)
    { myprintf("Damn ! Re-Generating a living animat !"); return; }
    space = dSimpleSpaceCreate(globalspace);
    idxgen = 0;
    transGenome();
   // printGenome(); printRepres();
   // displayRepres();
    readRepres (0, 0, 0, alpha);
    dGeomGetAABB ((dGeomID)space, aabb);
    trunkpos = (dReal *)dBodyGetPosition(limbs[0].id);
    setPos (nx, ny, (trunkpos[2]-aabb[4]) + 0.01);
    for (i=0; i < MAXLIMBS; i++)
	for (j = 0; j < MAXNEUR; j++)
	{
	    repres[i].neurons[j].state = 0;
	    repres[i].neurons[j].out=0;
	}
    for (i=0; i < MAXLIMBS; i++)
	limbs[i].oldspeed = 0; // doing this in the constructor has no effect!?
    i = 0; while ((regist[i]) && (i < REGISTSIZE)) i++;
    if (i >= REGISTSIZE)
	mydie("Damn ! Register full !\n");
    regist[i] = this;
    alive = 1;
}

Limb *Animat::createLimb (int index, dReal  zel, dReal zew, dReal zeh,
	dReal x, dReal y, dReal z, dReal alpha)
{
    // create a new limb, using parameters for morphological data

    Limb *res;
    dReal l, w, h;
    dMatrix3 R1;
    l = 0.2 + zel;
    w = 0.2 + zew;
    h = 0.2 + zeh;
    if (index >= MAXLIMBS)
	mydie("Damn ! Too many limbs !\n");
    res = &limbs[index];
    if (res->id)
	mydie("Damn ! Limb %d exists\n",index);
    res->lgt = l; res->wdt = w; res->hgt = h;
    res->id = dBodyCreate(world);
    res->index = index;
    res->type = Limb::TORSO;
    res->immunitytimer=0;
//    dMassSetBox (&res->mass,1, l / 2.0, w / 2.0, h / 2.0);
    dMassSetBox (&res->mass,1, l, w, h);
//    dMassAdjust(&res->mass, (l + w + h) / 10.0);
    dMassAdjust(&res->mass, (l+w+h)/3.0);
    res->alpha = 0; res->beta = 0;
    res->sides[0] = l; res->sides[1] = w; res->sides[2] = h;
    res->damage = 0;
    res->damagedone = 0;
    res->owner = this;

    res->geom = dCreateBox (space, l, w, h);

    // Each geom has a pointer to the corresponding limb, which is useful for
    // damage calculation
    dGeomSetData (res->geom, res);
    //myprintf("%d %d\n", index, res);

    dGeomSetBody (res->geom, res->id);

    dBodySetMass(res->id, &res->mass);
    dBodySetPosition   (res->id, x, y, z);
    dRFromAxisAndAngle (R1, 0, 0, 1, alpha);
    dBodySetRotation (res->id, R1);
    //dBodySetPosition (res->id, x, y, z);

    res->dad = 0;
    return res;
}

Limb *Animat::addLimb (Limb *trunk, int idxGen)
{
    // add a new limb, reading the data from the repres at position idxGen

    dReal alpha, beta; int orient;
    dReal size = 0;
    dReal *tmpmat;
    dQuaternion q1, q2, q3;
    dVector3 tmpvect;
    Limb *leg;

    /* discretization
    myprintf("\nOld alpha = %.3f\n", repres[idxGen].alpha);
    alpha = 4 * repres[idxGen].alpha;
    alpha = rint(alpha);
    alpha = alpha / 4.0;
    beta = 4 * repres[idxGen].beta;
    beta = rint(beta);
    beta = beta / 4.0;*/

    alpha =  2.0 * M_PI * repres[idxGen].alpha;
    beta =  2.0 * M_PI * repres[idxGen].beta;

    orient = repres[idxGen].orient;
    leg = createLimb(idxGen, repres[idxGen].lgt, repres[idxGen].wdt,
	    repres[idxGen].hgt, 0, 0, 0, 0);
    leg->alpha = alpha; leg->beta = beta;
//myprintf("%f \n", alpha);
    anglesToPoint(trunk->id, trunk->lgt, trunk->wdt, trunk->hgt,
	    alpha, beta, tmpvect);

    tmpmat = (dReal *) dBodyGetRotation (trunk->id);
    dQFromAxisAndAngle(q3, tmpmat[2], tmpmat[6], tmpmat[10], alpha);
    dRtoQ (dBodyGetRotation (trunk->id), q2);
    dQMultiply0 (q1, q3, q2);  // q1 = q3 * q2
    dBodySetQuaternion (leg->id, q1);
    tmpmat = (dReal *) dBodyGetRotation (leg->id);
    //dQFromAxisAndAngle (q3, -tmpmat[4], -tmpmat[5], -tmpmat[6], beta);
    dQFromAxisAndAngle (q3, -tmpmat[1], -tmpmat[5], -tmpmat[9], beta);
    dRtoQ (dBodyGetRotation (leg->id), q2);
    dQMultiply0 (q1, q3, q2);  // q1 = q2 * q3
    dBodySetQuaternion (leg->id, q1);


    dBodySetPosition (leg->id, tmpvect[0], tmpvect[1], tmpvect[2]);
    // We must push the leg half it length in its own x direction
    // So that it "only just" touches the trunk at contact point
    tmpmat = (dReal *) dBodyGetRotation (leg->id);
    size = (repres[idxGen].lgt + 0.2) / 2;
    tmpvect[0] += tmpmat[0]*size;
    tmpvect[1] += tmpmat[4]*size;
    tmpvect[2] += tmpmat[8]*size;
    dBodySetPosition (leg->id, tmpvect[0], tmpvect[1], tmpvect[2]);

    /* OLD !
    // Obtaining the coords of the vector between trunk center and anchor point
    *trunkpos = (dReal *) dBodyGetPosition (trunk->id);
    for (i=0; i < 3; i++) tmpvect[i] -= trunkpos[i];
    for (i=0; i < 3; i++) size += tmpvect[i] * tmpvect[i];
    size = sqrt(size);
    size = size / (leg->lgt / 2.0);
    for (i=0; i < 3; i++) tmpvect[i] = tmpvect[i] / size;
    limbpos = (dReal *) dBodyGetPosition (leg->id);
    for (i=0; i < 3; i++) tmpvect[i] = limbpos[i] + tmpvect[i];
    dBodySetPosition (leg->id, tmpvect[0], tmpvect[1], tmpvect[2]);*/


    anglesToPoint(trunk->id, trunk->lgt, trunk->wdt, trunk->hgt, alpha, beta, tmpvect);

    tmpmat = (dReal *) dBodyGetRotation (leg->id);
      leg->joint = dJointCreateHinge (world, 0);
    dJointAttach (leg->joint, trunk->id, leg->id);
    dJointSetHingeAnchor (leg->joint, tmpvect[0], tmpvect[1], tmpvect[2]);
    if (orient == 1)
    {
	if (repres[idxGen].symmetrised)
	    dJointSetHingeAxis (leg->joint, tmpmat[2], tmpmat[6], tmpmat[10]);
	else
	    dJointSetHingeAxis (leg->joint, -tmpmat[2], -tmpmat[6], -tmpmat[10]);
    }
    else
        dJointSetHingeAxis (leg->joint, tmpmat[1], tmpmat[5], tmpmat[9]);
    dJointSetHingeParam(leg->joint, dParamVel, 0);
    dJointSetHingeParam(leg->joint, dParamFMax, MAXFORCE);

    dJointSetHingeParam(leg->joint, dParamFudgeFactor, 0.3);
    dJointSetHingeParam(leg->joint, dParamLoStop, -0.75* M_PI);
    dJointSetHingeParam(leg->joint, dParamHiStop, 0.75* M_PI);

    leg->dad = trunk;
    leg->type = repres[idxGen].type;
    if(leg->type==Limb::HEAD)
        head = leg;
    return leg;
}

void Animat::readRepres(dReal x, dReal y, dReal z, dReal alpha)
{

    // create me within the simulation, by reading morphological data from the
    // repres (used by generate(), shouldn't be used directly)

    int idx;
    Limb *cur;
//    myprintf("Creating Dad ...\n");
    cur = createLimb(0, repres[0].lgt,
		repres[0].wdt,
		repres[0].hgt,
		x, y, z, alpha);
    for (idx=1; idx < MAXLIMBS; idx++)
    {
	if (repres[idx].dead) break;
	if (idx >= MAXLIMBS) break;
	cur = addLimb(&limbs[repres[idx].dadnum], idx);
//	myprintf("Adding Son (idx=%d)...\n", idx);
    }
    myprintf("Created, break! (idx=%d)...\n", idx);
}


//Amin: Global functions
void addDamage(Animat *A, Animat *B, dReal damage)
{
    // utility function: takes note that A deals damage to B
    for (int i=0; i < REGISTSIZE; i++)
    {
	if (!regist[i]) continue;
	if (regist[i] == A)
	    for (int j=0; j < REGISTSIZE; j++)
	    {
		if (!regist[j]) continue;
		if (regist[j] == B)
		    DAMAGETABLE[i][j] += damage;
	    }
    }
}


void resetScene()
{
    // reset the world
    int i;
    for (i=0; i < REGISTSIZE; i++)
    {
	if (regist[i])
	{
	   if (regist[i]->alive)
	       regist[i]->remove();
	   else
	       mydie("Damn ! Registered animat not alive !\n");
	}
	regist[i] = NULL;
    }
    dRandSetSeed(1); // dRandSetSeed sets the ODE-specific random seed
    if (BALL)
    {
	ball.remove();
	ball.generate();
    }
}


void initWorld()
{

    // create the world
    //
    // must always be called at the start of a program

    int i;
    FILE *randfile;
    unsigned int seed;
    for (i=0; i < REGISTSIZE; i++)
	regist[i] = NULL;
    //srandom(MYRANDSEED);

    if (MYRANDSEED == 0)
    {
	randfile = fopen("/dev/urandom", "r");
	fread (&seed, sizeof(seed), 1, randfile);
	fclose(randfile);
    }
    else seed = MYRANDSEED;
    srandom(seed);
    myprintf("Random seed is %u\n", seed);

    // setup pointers to drawstuff callback functions
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.stop = 0;

    // NOTE: If you ARE using the modified drawstuff.cpp file, uncomment this
    // to prevent the ground from being drawn.

    //dsSetDrawGround(0);
    //dsSetShadows(0);


    // NOTE: If you ARE NOT using the modified drawstuff.cpp file, you will
    // need to set this directory to wherever the texture files are.
    fn.path_to_textures = ".";
    //fn.path_to_textures = "/home/pg/txm/ode-0.5/drawstuff/textures";

    dsSetSphereQuality(3);

    // create world
    world = dWorldCreate();
    globalspace = dSimpleSpaceCreate (0);
    alternatespace = dSimpleSpaceCreate (0);
    contactgroup = dJointGroupCreate (0);
    dRSetIdentity(IDENTITY);
    dWorldSetCFM (world, MYCFM);//0.05 // was 1e-5 - too low // .1 bad
    dWorldSetERP (world, MYERP); // should NOT be 0.5+ (too springy)*/
    tot_time = 0;
    numround = 0;
    if (WORLDTYPE == SPHERICWORLD)
    {
	dWorldSetGravity (world,0,0,0);
	sphereID = dCreateSphere (globalspace, WORLDRADIUS);
	groundID = NULL;
	dGeomSetPosition(sphereID, 0, 0, 0);
    }
    else if (WORLDTYPE == FLATWORLD)
    {
	dWorldSetGravity (world,0,0,-9.8);
	groundID = dCreatePlane (globalspace,0,0,1,0);
	sphereID = NULL;
    }
    else {
	myprintf("You must specify whether the world is spherical or flat!\n");
	myprintf("Please set the value of WORLDTYPE to SPHERICWORLD or FLATWORLD before calling initWorld()\n");
	mydie("Damn! World is neither spheric nor flat!\n");
    }

    if (BOARD) makeBoard();
    if (WALLS) makeWalls();
    if (CORRIDOR) makeCorridor();
    //if (BALL) ball.generate();
    // propriete des surfaces de contact des boites (voir test_boxstack.cpp)
    for (i=0; i< MAXCONT; i++) {
//    contact[i].surface.mode = dContactApprox1|dContactSoftCFM;
//    contact[i].surface.mode = dContactApprox1;
    contact[i].surface.mu = 1.0;
//    contact[i].surface.soft_cfm = 0.01;
    }


}

// Callback for collisions
void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  //if ((o1 != sphereID) && (o2 != sphereID)) return;

  if (dGeomIsSpace (o1) || dGeomIsSpace (o2))
  {
    //if ((o1 == sphereID) || (o2 == sphereID)) myprintf("Ground\n");
      dSpaceCollide2 (o1,o2,data,&nearCallback);
      if (dGeomIsSpace (o1)) dSpaceCollide ((dxSpace*)o1,data,&nearCallback);
      if (dGeomIsSpace (o2)) dSpaceCollide ((dxSpace*)o2,data,&nearCallback);
  }
  else
  {
      //myprintf("Collide\n");


      // if the two bodies are already connected by
      //  any type of joint, whether hinge or contact, we skip this pair -
      //  barbarian style.
      //  It makes damage calculation much simpler.


      // the first conditional test is necessary because if either o1 or o2 is
      // a non-body geom (i.e. the ground or sphere geom), the debug build of
      // ODE crashes out with message: ODE INTERNAL ERROR 2: Bad argument(s)
      // in dAreConnected()


      int numc=dCollide(o1,o2,MAXCONT,&contact[0].geom,sizeof(dContact));
      dJointID JntID;
      
      if (numc )
      {
          // Damage calculation

          if(o1==groundID||o2==groundID)
          {
              Limb *l= (o1==groundID)?(Limb*) dGeomGetData(o2):(Limb*) dGeomGetData(o1);

              //myprintf("\nFall!\n");
	      if (!l)
		  mydie("Damn ! No limb data associated to bodies !\n");

              for (int i=0; i<numc; i++)
              {
                  JntID = dJointCreateContact (world,contactgroup,contact+i);
                  dJointAttach (JntID,l->id,0);                          
              }

              l->collisionDamage(false);
	  }
          else
          {
              Limb *l1= (Limb*) dGeomGetData(o1),
                   *l2= (Limb*) dGeomGetData(o2);
              if(l1&&l2)
              {
                  for (int i=0; i<numc; i++)
                  {
                      JntID = dJointCreateContact (world,contactgroup,contact+i);
                      dJointAttach (JntID,l1->id,l2->id);
                  }
              //    l1->collisionDamage(true);
              //    l2->collisionDamage(true);
              }
          }

	  /*if (VISUAL)
	  {
	       draw little red cubes at contact points
	      if (numc > 0) {
		  for (int i=0; i<numc; i++) {
		      dsSetColor(1,0,0);
		      dsDrawBox (contact[i].geom.pos,IDENTITY,ss);
		  }
	      }
	  }*/
      }
  }
}

dReal Animat::height()
{
    if(!head)
    {
        cout<<"Headless!\n";
        exit(-1);
    }
    else
        return *(dBodyGetPosition(head->id)+2);//height of head
}

void Animat::update(dReal stepLength,dReal minHeight,dReal z_stretch)//,dReal progress)
{

    nbLimbs = nblimbs();
    for(int i=0;i<nbLimbs;i++)
    {
        limbs[i].updateVel(stepLength);
        if(limbs[i].type==Limb::HEAD)
        {
            const dReal *pos = dBodyGetPosition(limbs[i].id);
            if(oldX==-1&&oldY==-1)
            {oldX=pos[0];oldY=pos[1];first_height=pos[2];}
            else
            {
                vect2d motion(pos[0]-oldX,pos[1]-oldY);
                //cout<<"progress "<<progress;
                //printf("old(%f,%f) new(%f,%f) height(%f)",oldX,oldY,pos[0],pos[1],pos[2]);
                //motion.print(" - motion");
                //motionSum.print("sum");
                if((motion&motion)<1)//The first result of dBodyGetPosition seems to be always wrong (about 47 motion in x direction)
                {
                    dReal h = (pos[2]-minHeight)*z_stretch;
                    if(h>0&&h>first_height*0.8)
                    {
                        motionSum+=motion;
                        Motion.push(motion);
                        Height.push(h);
                    }
                }
                oldX=pos[0];oldY=pos[1];
            }
         
        }
    }
    
}

dReal Animat::damage()
{
    nbLimbs = nblimbs();
    dReal Damage = 0.0;
    dReal max1=0,max2=0;

    for(int i=0;i<nbLimbs;i++)
    {
        Damage +=(limbs[i].type==Limb::HEAD)?limbs[i].damage*10:limbs[i].damage;
        if(limbs[i].type==Limb::LIMB)
        {
            if(max1<limbs[i].damage)
            {
                max2=max1;
                max1=limbs[i].damage;
                continue;
            }
            if(max2<limbs[i].damage)
                max2=limbs[i].damage;
        }
    }
    return (Damage-max1-max2)*1e-4;
}

dReal Animat::score()
{
    if(scoreVector.empty())
        return 0.0;

    return scoreVector.back();
}

dReal Animat::newScore(dReal damage_coef)
{
    dReal motion_height_score=0.0,ret;
    
    while(!Motion.empty())
    {
        motion_height_score+=Height.top()*Motion.top().projection_on(motionSum);
        Motion.pop();Height.pop();
    }
    ret = motion_height_score/(1+damage()*damage_coef);
    scoreVector.push_back(ret);
    return ret;
}

dReal Animat::averageScore(bool current)
{
    if(avg_score!=0.0&&current)
        return avg_score;

   if(!scoreVector.size())
        return 0.0;

    dReal ret=0.0;
    for(vector<dReal>::iterator it = scoreVector.begin();it<scoreVector.end();it++)
        ret+=*it;
    avg_score = ret/scoreVector.size();
    return avg_score;
}