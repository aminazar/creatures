/* 
 * File:   limb.h
 * Author: amin
 *
 * Created on 03 April 2010, 01:47
 */

#ifndef _LIMB_H
#define	_LIMB_H
#include "definitions.h"


class Animat;
class Limb {

public:
    enum LimbType {HEAD,TORSO,LIMB};
    static double DAMAGE_THRESHOLD;
    static double BODY_DAMAGE_THRESHOLD;
    void updateVel(dReal step);
    dReal velocity[3],angular_velocity[3],acceleration[3],angular_acceleration[3];
    dBodyID id;
    dJointID joint;
    dGeomID geom;
    int index;
    dReal lgt;
    int touchesOther;
    dReal oldspeed;
    dReal wdt;
    dReal hgt;
    Animat *owner;
    int immunitytimer;
    LimbType type;
    dReal alpha, beta;
    dReal damage, damagedone, curdamage;
    dReal sides[3];
    dMatrix3 oldrot;
    dReal oldpos[3];
    dMass mass;
    Limb *dad;
    Limb();
    void reset();
    void collisionDamage(bool inside);
};

#endif	/* _LIMB_H */

