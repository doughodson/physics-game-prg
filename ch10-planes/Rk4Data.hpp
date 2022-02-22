#ifndef __Rk4Data_HPP__
#define __Rk4Data_HPP__

//-----------------------------------------------------
// Structure: Rk4Data
//-----------------------------------------------------
struct Rk4Data
{
	int numEqns{};
	double q[6];    // vx, x, vy, y, vz, z
};

#endif
