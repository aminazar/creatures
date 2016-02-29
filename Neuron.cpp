/* 
 * File:   Neuron.cpp
 * Author: amin
 * 
 * Created on 03 April 2010, 00:52
 */

#include "Neuron.h"

extern int SENSORTYPES[NBSENSORTYPES];

Neuron::Neuron() {
    int i;
	type = INTER;
	function = TANH;
	exists = 0;
	for (i=0; i < MAXCONFROM; i++)
	{
	    confrom[i].exists = 0;
	    confrom[i].limb = -2;
	    confrom[i].neur= -2;
	    confrom[i].val = 0;
	    confrom[i].rectype = RECSELF;
	    confrom[i].reftype = REFBOTH;
	}
	threshold = 0; out = 0; state = 0;
    }
/*
Neuron::Neuron(const Neuron& orig) {
}*/


void Neuron::randVals()
        {
	threshold = MAXTHRES * (double)(random() % 1000) / 1000.0;
	if (random() % 2) threshold = -threshold;
	/*switch (random() % 3)
	{
	    case 0:
		function = INVEXP; break;
	    case 1:
		function = SIGMOID; break;
	    case 2:
		function = TANH; break;
	}*/
	//function = SIGMOID;
	switch (random() % 2)
	{
	    case 0:
		function = TANH; break;
	    case 1:
		function = SIGMOID; break;
	}
    }

int Neuron::isExtSensor()

    {
    for (int i=0; i < NBSENSORTYPES; i++)
	if (type == SENSORTYPES[i])
	    return 1;
    return 0;
    }