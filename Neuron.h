/* 
 * File:   Neuron.h
 * Created on 03 April 2010, 00:52
 */

#ifndef _NEURON_H
#define	_NEURON_H
#include <ode/ode.h>
#include "definitions.h"
class Neuron
{
    public:
    int exists; // 0 / 1
    double state;
    double threshold; // -3/2..3/2
    double out;  // = sigmoid or mytanh(state+threshold)
    int type;
    int function;
    struct{ int exists; int limb; int neur; int rectype; int reftype; double val;}
    confrom[MAXCONFROM];
    void randVals();
    int isExtSensor();

    Neuron();

};


#endif	/* _NEURON_H */

