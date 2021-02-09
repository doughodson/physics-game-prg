
#include <cstdlib>
#include <cstring>
#include <cmath>

#include "Plane.hpp"

//*************************************************************
//  This method loads the right-hand sides for the plane ODEs
//*************************************************************
void planeRightHandSide(Plane* plane,
   double* q, double* deltaQ, double ds,
   double qScale, double* dq) {
   //  q[0] = vx = dxdt
   //  q[1] = x
   //  q[2] = vy = dydt
   //  q[3] = y
   //  q[4] = vz = dzdt
   //  q[5] = z
   double newQ[6]; // intermediate dependent variable values.

   double bank;
   double alpha;
   double throttle;
   double wingArea;
   double wingSpan;
   double tailArea;
   double clSlope0;
   double cl0;
   double clSlope1;
   double cl1;
   double alphaClMax;
   double cdp;
   double eff;
   double mass;
   double enginePower;
   double engineRps;
   double propDiameter;
   double a;
   double b;

   int i;
   double pi = acos(-1.0);
   double G = -9.81;

   double vx;
   double vy;
   double vz;
   double x;
   double y;
   double z;
   double vh;
   double vtotal;
   double temperature;
   double grp;
   double pressure;
   double density;
   double omega;
   double factor;
   double advanceRatio;
   double thrust;
   double cl;
   double lift;
   double aspectRatio;
   double cd;
   double drag;
   double cosW;   //  bank angle
   double sinW;   //  bank angle
   double cosP;   //  climb angle
   double sinP;   //  climb angle
   double cosT;   //  heading angle
   double sinT;   //  heading angle
   double Fx;
   double Fy;
   double Fz;

   bank = plane->bank;
   alpha = plane->alpha;
   throttle = plane->throttle;
   wingArea = plane->wingArea;
   wingSpan = plane->wingSpan;
   tailArea = plane->tailArea;
   clSlope0 = plane->clSlope0;
   cl0 = plane->cl0;
   clSlope1 = plane->clSlope1;
   cl1 = plane->cl1;
   alphaClMax = plane->alphaClMax;
   cdp = plane->cdp;
   eff = plane->eff;
   mass = plane->mass;
   enginePower = plane->enginePower;
   engineRps = plane->engineRps;
   propDiameter = plane->propDiameter;
   a = plane->a;
   b = plane->b;

   //  Convert bank angle from degrees to radians
   //  Angle of attack is not converted because the
   //  Cl-alpha curve is defined in terms of degrees.
   bank = bank * pi / 180.0;

   //  Compute the intermediate values of the 
   //  dependent variables.
   for (i = 0; i < 6; ++i) {
      newQ[i] = q[i] + qScale * deltaQ[i];
   }

   //  Assign convenenience variables to the intermediate 
   //  values of the locations and velocities.
   vx = newQ[0];
   vy = newQ[2];
   vz = newQ[4];
   x = newQ[1];
   y = newQ[3];
   z = newQ[5];
   vh = sqrt(vx * vx + vy * vy);
   vtotal = sqrt(vx * vx + vy * vy + vz * vz);

   //  Compute the air density
   temperature = 288.15 - 0.0065 * z;
   grp = (1.0 - 0.0065 * z / 288.15);
   pressure = 101325.0 * pow(grp, 5.25);
   density = 0.00348 * pressure / temperature;

   //  Compute power drop-off factor
   omega = density / 1.225;
   factor = (omega - 0.12) / 0.88;

   //  Compute thrust
   advanceRatio = vtotal / (engineRps * propDiameter);
   thrust = throttle * factor * enginePower *
      (a + b * advanceRatio * advanceRatio) / (engineRps * propDiameter);

   //  Compute lift coefficient. The Cl curve is 
   //  modeled using two straight lines.
   if (alpha < alphaClMax) {
      cl = clSlope0 * alpha + cl0;
   }
   else {
      cl = clSlope1 * alpha + cl1;
   }

   //  Include effects of flaps and ground effects.
   //  Ground effects are present if the plane is
   //  within 5 meters of the ground.
   if (!std::strcmp(plane->flap, "20")) {
      cl += 0.25;
   }
   if (!std::strcmp(plane->flap, "40")) {
      cl += 0.5;
   }
   if (z < 5.0) {
      cl += 0.25;
   }

   //  Compute lift
   lift = 0.5 * cl * density * vtotal * vtotal * wingArea;

   //  Compute drag coefficient
   aspectRatio = wingSpan * wingSpan / wingArea;
   cd = cdp + cl * cl / (pi * aspectRatio * eff);

   //  Compute drag force
   drag = 0.5 * cd * density * vtotal * vtotal * wingArea;

   //  Define some shorthand convenience variables
   //  for use with the rotation matrix.
   //  Compute the sine and cosines of the climb angle,
   //  bank angle, and heading angle;
   cosW = std::cos(bank);
   sinW = std::sin(bank);

   if (vtotal == 0.0) {
      cosP = 1.0;
      sinP = 0.0;
   }
   else {
      cosP = vh / vtotal;
      sinP = vz / vtotal;
   }

   if (vh == 0.0) {
      cosT = 1.0;
      sinT = 0.0;
   }
   else {
      cosT = vx / vh;
      sinT = vy / vh;
   }

   //  Convert the thrust, drag, and lift forces into
   //  x-, y-, and z-components using the rotation matrix.
   Fx = cosT * cosP * (thrust - drag) +
      (sinT * sinW - cosT * sinP * cosW) * lift;
   Fy = sinT * cosP * (thrust - drag) +
      (-cosT * sinW - sinT * sinP * cosW) * lift;
   Fz = sinP * (thrust - drag) + cosP * cosW * lift;

   //  Add the gravity force to the z-direction force.
   Fz = Fz + mass * G;

   //  Since the plane can't sink into the ground, if the
   //  altitude is less than or equal to zero and the z-component
   //  of force is less than zero, set the z-force
   //  to be zero.
   if (z <= 0.0 && Fz <= 0.0) {
      Fz = 0.0;
   }

   //  Load the right-hand sides of the ODE's
   dq[0] = ds * (Fx / mass);
   dq[1] = ds * vx;
   dq[2] = ds * (Fy / mass);
   dq[3] = ds * vy;
   dq[4] = ds * (Fz / mass);
   dq[5] = ds * vz;

   return;
}

//************************************************************
//  This method solves for the plane motion using a
//  4th-order Runge-Kutta solver
//************************************************************
void planeRungeKutta4(Plane *plane, double ds) {

  int j;
  int numEqns;
  double s;
  double *q;
  double *dq1;
  double *dq2;
  double *dq3;
  double *dq4;

  //  Define a convenience variable to make the
  //  code more readable
  numEqns = plane->numEqns;

  //  Allocate memory for the arrays.
  q = (double *)std::malloc(numEqns*sizeof(double));
  dq1 = (double *)std::malloc(numEqns*sizeof(double));
  dq2 = (double *)std::malloc(numEqns*sizeof(double));
  dq3 = (double *)std::malloc(numEqns*sizeof(double));
  dq4 = (double *)std::malloc(numEqns*sizeof(double));

  //  Retrieve the current values of the dependent
  //  and independent variables.
  s = plane->s;
  for(j=0; j<numEqns; ++j) {
    q[j] = plane->q[j];
  }     

  // Compute the four Runge-Kutta steps, The return 
  // value of planeRightHandSide method is an array
  // of delta-q values for each of the four steps.
  planeRightHandSide(plane, q, q,   ds, 0.0, dq1);
  planeRightHandSide(plane, q, dq1, ds, 0.5, dq2);
  planeRightHandSide(plane, q, dq2, ds, 0.5, dq3);
  planeRightHandSide(plane, q, dq3, ds, 1.0, dq4);

  //  Update the dependent and independent variable values
  //  at the new dependent variable location and store the
  //  values in the ODE object arrays.
  plane->s = plane->s + ds;

  for(j=0; j<numEqns; ++j) {
    q[j] = q[j] + (dq1[j] + 2.0*dq2[j] + 2.0*dq3[j] + dq4[j])/6.0;
    plane->q[j] = q[j];
  }     

  //  Free up memory
  std::free(q);
  std::free(dq1);
  std::free(dq2);
  std::free(dq3);
  std::free(dq4);

  return;
}
 
