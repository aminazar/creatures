#include "limb.h"
#include "definitions.h"

double Limb::DAMAGE_THRESHOLD=0.01;
double Limb::BODY_DAMAGE_THRESHOLD=0.1;

Limb::Limb()
{
    id = 0; 
    joint=0; 
    oldspeed = 0;
    reset();
}

void Limb::reset()
{
    for(int i=0;i<3;i++)
    {
        velocity[i]=0.0;
        angular_velocity[i]=0.0;
        acceleration[i]=0.0;
        angular_acceleration[i]=0.0;

    }
    damage = 0;
}

void Limb::updateVel(dReal step)
{
  dReal tmpVel[3],tmpAngVel[3];
  if(id)for(int i=0;i<3;i++)
  {
      tmpVel[i]=velocity[i];
      velocity[i]      = *(dBodyGetLinearVel(id)+i);
      tmpAngVel[i]=angular_velocity[i];
      angular_velocity[i]   = *(dBodyGetAngularVel(id)+i);
      if(step!=0.0)
      {
          acceleration[i]=(velocity[i]-tmpVel[i])/step;
          angular_acceleration[i]=(angular_velocity[i]-tmpAngVel[i])/step;
          //print_3d_vect(acceleration,"acceleration");
          //print_3d_vect(angular_acceleration,"angular acceleration");
      }
  }
}

void Limb::collisionDamage(bool inside)
{//Calculates damage in the moment of collision (inside the simulation)
    dReal delta_linear_momentum = 0.0,
          delta_angular_momentum= 0.0;
    //double threshold = inside?BODY_DAMAGE_THRESHOLD:DAMAGE_THRESHOLD;
    for(int i=0;i<3;i++)
    {

        delta_linear_momentum = *(dBodyGetLinearVel(id)+i)*velocity[i]*mass.mass;
              
        for(int j=0;j<3;j++)
            delta_angular_momentum = *(dBodyGetAngularVel(id)+j)*angular_velocity[j]*mass.I[i*4+j];
    
         if(delta_linear_momentum<-DAMAGE_THRESHOLD)
        {
            //  myprintf("linear mom is negative:%f\n",delta_linear_momentum);
            damage -= (1/DAMAGE_THRESHOLD)*delta_linear_momentum;
        }

        if(delta_angular_momentum<-DAMAGE_THRESHOLD)
        {
            //printf("angular mom is negative:%f\n",delta_angular_momentum);
            damage -= (1/DAMAGE_THRESHOLD)*delta_angular_momentum;
        }
    
    }
}