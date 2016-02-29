/* 
 * File:   gene.h
 * Author: amin
 *
 * Created on 03 April 2010, 01:26
 */

#ifndef _GENE_H
#define	_GENE_H
#include "definitions.h"
#include "Neuron.h"
#include "limb.h"
#include "stocc.h"

extern StochasticLib1 sto;
class gene
{
    // each gene represents one limb, both morph. and neural info
    public:
    static const int TORSO_COEFF=6;
    int dead;
    int dadnum; // the "dad" gene to this gene
    int terminal;
    int recur; // recursive symmetry (within [0..MAXRECUR] range)
    int symYinputs; // not used
    double lgt;
    double wdt;
    double hgt;
    Limb::LimbType type;
    int symmetrised; // set by the developmental system
    int origgene; // in repres, original num in genome
    double alpha, beta; // angles from ancestor limb
    double alpharec, betarec; // angles for recursion - NOT USED AT PRESENT!
    int orient; // orientation: "vertical" or "horizontal"
    int ref1, ref2, ref3; // bilateral symmetry (only ref1 is used)
    Neuron neurons[MAXNEUR];
    gene();
    //gene(const gene& orig);
    void randBodyGene(int dad,Limb::LimbType type=Limb::LIMB);
};

#endif	/* _GENE_H */

