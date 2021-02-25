#include <cmath>
#include <cstdio>

#include "plane.hpp"

void eom(Plane* plane, const double dt);

//-----------------------------------------------------
// initializes a plane and solves for the plane motion using the Runge-Kutta solver
//-----------------------------------------------------
int main(int argc, char *argv[])
{
  Plane plane;

  // Cessna 172 characteristics
  plane.wingArea = 16.2;        // wing wetted area, m^2
  plane.wingSpan = 10.9;        // wing span, m
  plane.tailArea = 2.0;         // tail wetted area, m^2
  plane.clSlope0 = 0.0889;      // slope of Cl-alpha curve
  plane.cl0 = 0.178;            // Cl value when alpha = 0
  plane.clSlope1 = -0.1;        // slope of post-stall Cl-alpha curve
  plane.cl1 = 3.2;              // intercept of post-stall Cl-alpha curve
  plane.alphaClMax = 16.0;      // alpha at Cl(max)
  plane.cdp = 0.034;            // parasitic drag coefficient
  plane.eff = 0.77;             // induced drag efficiency coefficient
  plane.mass = 1114.0;          // airplane mass, kg
  plane.enginePower = 119310.0; // peak engine power, W
  plane.engineRps = 40.0;       // engine turnover rate, rev/s
  plane.propDiameter = 1.905;   // propeller diameter, m
  plane.a = 1.83;               // propeller efficiency curve fit coefficient
  plane.b = -1.32;              // propeller efficiency curve fit coefficient

  plane.bank = 0.0;
  plane.alpha = 4.0;
  plane.throttle = 1.0;
  plane.flap = "0";             //  flap setting

  plane.numEqns = 6;
  plane.time = 0.0;   //  time 
  plane.q[0] = 0.0;   //  vx 
  plane.q[1] = 0.0;   //  x  
  plane.q[2] = 0.0;   //  vy 
  plane.q[3] = 0.0;   //  y  
  plane.q[4] = 0.0;   //  vz 
  plane.q[5] = 0.0;   //  z  

  // execute simulation for 40 seconds
  const double dt{ 0.5 };
  while ( plane.time < 40.0 ) {
    eom(&plane, dt);

    const double x{plane.q[1]};
    const double z{plane.q[5]};
    const double v{std::sqrt(plane.q[0]*plane.q[0] + plane.q[2]*plane.q[2] + plane.q[4]*plane.q[4])};

    std::printf("time=%lf x=%lf  altitude=%lf  airspeed=%lf\n", plane.time, x, z, v);
  }

  return 0;
}

