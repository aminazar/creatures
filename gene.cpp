/* 
 * File:   gene.cpp
 * Author: amin
 * 
 * Created on 03 April 2010, 01:26
 */
#include "gene.h"

gene::gene() {
	dadnum = 0; dead = 0; lgt = 0; wdt=0; hgt=0; alpha=0; beta=0;
	alpharec=0; betarec=0;
	origgene = 0; ref1=0; ref2=0; ref3=0; recur=0; symmetrised=0;
	terminal=0;
    }

//gene::gene(const gene& orig) {}

//gene::~gene() {}

void gene::randBodyGene(int dad,Limb::LimbType Type)

    {
	// assigns random values, but only to morphology info (i.e. not neurons
	// info)
	dead = 0;
	dadnum = dad;
        type=Type;
	ref1 = 0; ref2 = 0; ref3 = 0;
        int coeff=(type==Limb::TORSO?TORSO_COEFF:1);
	lgt = coeff*(float)(sto.IRandomX(0, 10000)) / 10000.0;
	wdt = (type==Limb::LIMB?TORSO_COEFF:coeff)*(float)(sto.IRandomX(0,10000)) / 10000.0;
        hgt = (float)(sto.IRandomX(0, 10000)) / 10000.0;
	alpha = double (sto.IRandomX(0,7)  - 4) / 8.0; // [-4/8..3/8]
	beta = double (sto.IRandomX(0,7)  - 4) / 8.0; // [-4/8..3/8]
	//beta = 1.0 - (float)(sto.Random() % 2000) / 1000.0;
	// when in recursion, alpha = alpharec / 4
	//		      beta = betarec / 4
	//alpharec = 1.0 - (float)(sto.Random() % 2000) / 1000.0;
	//betarec = 1.0 - (float)(sto.Random() % 2000) / 1000.0;
	alpharec = double (sto.IRandomX(0,7)  - 4) / 8.0; // [-4/8..3/8]
	betarec = double (sto.IRandomX(0,7)  - 4) / 8.0; // [-4/8..3/8]
	symmetrised = 0;
	terminal = sto.IRandomX(0,1);
	if (sto.IRandomX(0,100) < PROBAINITRECUR && Type==Limb::LIMB)
            recur = sto.IRandomX(0,MAXRECUR);
        else
            recur=0;
	if (dad == -1)
           recur = 0;
        
	if (sto.IRandomX(0,100) < PROBAINITREF && Type==Limb::LIMB)
	    ref1 = 1;
        else ref1 = 0;
	if (sto.IRandomX(0,1)) orient = 1; else orient = -1;
    }

