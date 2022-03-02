#include <cmath>
#include <cstdio>

#include "Cessna172Properties.hpp"
#include "CtrlInputs.hpp"
#include "Rk4Data.hpp"

void eom_rk4(const Cessna172Properties&, const CtrlInputs&, Rk4Data*, const double dt);

//-----------------------------------------------------
// initializes a plane and solves for the plane motion using the Runge-Kutta solver
//-----------------------------------------------------
int main(int argc, char *argv[])
{
  Cessna172Properties prop;
  CtrlInputs inputs;

  Rk4Data rk4_data;
  rk4_data.numEqns = 6;
  rk4_data.q[0] = 0.0;   //  vx 
  rk4_data.q[1] = 0.0;   //  x  
  rk4_data.q[2] = 0.0;   //  vy 
  rk4_data.q[3] = 0.0;   //  y  
  rk4_data.q[4] = 0.0;   //  vz 
  rk4_data.q[5] = 0.0;   //  z  

  // execute simulation for 40 seconds
  const double dt{0.5};
  double time{};
  while (time < 40.0) {
    eom_rk4(prop, inputs, &rk4_data, dt);

    const double x{rk4_data.q[1]};
    const double z{rk4_data.q[5]};
    const double v{std::sqrt(rk4_data.q[0]*rk4_data.q[0] +
                             rk4_data.q[2]*rk4_data.q[2] +
                             rk4_data.q[4]*rk4_data.q[4])};

    std::printf("time=%lf x=%lf  altitude=%lf  airspeed=%lf\n", time, x, z, v);
    time += dt; // update simulation time
  }

  return 0;
}

