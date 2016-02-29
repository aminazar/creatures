/* 
 * File:   Ball.h
 * Author: amin
 *
 * Created on 03 April 2010, 01:41
 */

#ifndef _BALL_H
#define	_BALL_H
#include "definitions.h"

class Ball {
public:
    dBodyID body;
    dJointID joint;
    dGeomID geom;
    dReal sides[3];
    dMass mass;
    int alive;
    void generate();
    void remove();
    void draw();
    void setPos(dReal x, dReal y);
    void initPos();
};


#endif	/* _BALL_H */

