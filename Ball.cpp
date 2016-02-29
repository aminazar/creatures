/* 
 * File:   Ball.cpp
 * Author: amin
 * 
 * Created on 03 April 2010, 01:41
 */

#include "Ball.h"

extern int BALL;
extern dWorldID world;
extern dSpaceID globalspace;

void Ball::generate()
    {
	sides[0] = 0.5 ;
	sides[1] = 0.5 ;
	sides[2] = 0.5 ;
	BALL = 1;
	dMassSetBox(&mass, 1.0, sides[0], sides[1], sides[2]);
	body = dBodyCreate(world);
	dBodySetMass (body, &mass);
	geom = dCreateBox(globalspace, sides[0], sides[1], sides[2]);
	dGeomSetBody (geom, body);
	initPos();
	alive = 1;
    }

void Ball::remove()
    {
	dBodyDestroy (body); dGeomDestroy (geom);
	alive = 0;
	BALL = 0;
    }

void Ball::draw()
    {
	dsSetColor(0,0,0);
	dsDrawBox (dGeomGetPosition(geom), dGeomGetRotation(geom), sides);
    }

void Ball::setPos(dReal x, dReal y)
    {
	dMatrix3 mat;
	dBodySetPosition(body, x, y, sides[2]/2.0);
	dBodySetLinearVel(body, 0, 0, 0);
	dBodySetAngularVel(body, 0, 0, 0);
	dRSetIdentity(mat);
	dBodySetRotation(body, mat);
    }

void Ball::initPos()

    {
	dMatrix3 mat;
	dBodySetPosition(body, 0, 0, sides[2]/2.0);
	dBodySetLinearVel(body, 0, 0, 0);
	dBodySetAngularVel(body, 0, 0, 0);
	dRSetIdentity(mat);
	dBodySetRotation(body, mat);
    }