
#include <cstdlib>
#include <cstring>
#include <cmath>

#include "Plane.hpp"

const double pi{std::acos(-1.0)};
const double G{-9.81};

//-----------------------------------------------------
// loads the right-hand sides for the plane ODEs
//-----------------------------------------------------
void planeRightHandSide(Plane* plane, double* q, double* deltaQ, double ds, double qScale, double* dq)
{
   const double alpha{plane->alpha};
   const double throttle{plane->throttle};
   const double wingArea{plane->wingArea};
   const double wingSpan{plane->wingSpan};
   const double tailArea{plane->tailArea};
   const double clSlope0{plane->clSlope0};
   const double cl0{plane->cl0};
   const double clSlope1{plane->clSlope1};
   const double cl1{plane->cl1};
   const double alphaClMax{plane->alphaClMax};
   const double cdp{plane->cdp};
   const double eff{plane->eff};
   const double mass{plane->mass};
   const double enginePower{plane->enginePower};
   const double engineRps{plane->engineRps};
   const double propDiameter{plane->propDiameter};
   const double a{plane->a};
   const double b{plane->b};

   // convert bank angle from degrees to radians
   // angle of attack is not converted because the
   // Cl-alpha curve is defined in terms of degrees
   const double bank{plane->bank * pi / 180.0};

   // compute the intermediate values of the dependent variables
   double newQ[6]{};
   for (int i = 0; i < 6; ++i) {
      newQ[i] = q[i] + qScale * deltaQ[i];
   }

   // assign convenenience variables to the intermediate 
   // values of the locations and velocities
   const double vx{newQ[0]};
   const double vy{newQ[2]};
   const double vz{newQ[4]};
   const double x{newQ[1]};
   const double y{newQ[3]};
   const double z {newQ[5]};
   const double vh{std::sqrt(vx * vx + vy * vy)};
   const double vtotal{std::sqrt(vx * vx + vy * vy + vz * vz)};

   // air density
   const double temperature{288.15 - 0.0065 * z};
   const double grp{(1.0 - 0.0065 * z / 288.15)};
   const double pressure{101325.0 * std::pow(grp, 5.25)};
   const double density{0.00348 * pressure / temperature};

   // power drop-off factor
   const double omega{density / 1.225};
   const double factor{(omega - 0.12) / 0.88};

   // compute thrust
   const double advanceRatio{vtotal / (engineRps * propDiameter)};
   const double thrust{throttle * factor * enginePower * (a + b * advanceRatio * advanceRatio) / (engineRps * propDiameter)};

   // compute lift coefficient - the Cl curve is modeled using two straight lines
   double cl{};
   if (alpha < alphaClMax) {
      cl = clSlope0 * alpha + cl0;
   } else {
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

   // compute lift
   const double lift{0.5 * cl * density * vtotal * vtotal * wingArea};

   // compute drag coefficient
   const double aspectRatio{wingSpan * wingSpan / wingArea};
   const double cd{cdp + cl * cl / (pi * aspectRatio * eff)};

   // compute drag force
   const double drag{0.5 * cd * density * vtotal * vtotal * wingArea};

   // define some shorthand convenience variables for use with the rotation matrix
   // compute the sine and cosines of the climb angle, bank angle, and heading angle
   const double cosW{std::cos(bank)};
   const double sinW{std::sin(bank)};

   double cosP{};   //  climb angle
   double sinP{};   //  climb angle
   double cosT{};   //  heading angle
   double sinT{};   //  heading angle

   if (vtotal == 0.0) {
      cosP = 1.0;
      sinP = 0.0;
   } else {
      cosP = vh / vtotal;
      sinP = vz / vtotal;
   }

   if (vh == 0.0) {
      cosT = 1.0;
      sinT = 0.0;
   } else {
      cosT = vx / vh;
      sinT = vy / vh;
   }

   // convert the thrust, drag, and lift forces into x-, y-, and z-components using the rotation matrix
   const double Fx{cosT * cosP * (thrust - drag) + (sinT * sinW - cosT * sinP * cosW) * lift};
   const double Fy{sinT * cosP * (thrust - drag) + (-cosT * sinW - sinT * sinP * cosW) * lift};
   double Fz{sinP * (thrust - drag) + cosP * cosW * lift};

   // add the gravity force to the z-direction force.
   Fz = Fz + mass * G;

   // since the plane can't sink into the ground, if the altitude is less than or equal to zero and the z-component
   // of force is less than zero, set the z-force to be zero
   if (z <= 0.0 && Fz <= 0.0) {
      Fz = 0.0;
   }

   // load the right-hand sides of the ODE's
   dq[0] = ds * (Fx / mass);
   dq[1] = ds * vx;
   dq[2] = ds * (Fy / mass);
   dq[3] = ds * vy;
   dq[4] = ds * (Fz / mass);
   dq[5] = ds * vz;

   return;
}

//-----------------------------------------------------
// 4th-order Runge-Kutta solver for plane motion
//-----------------------------------------------------
void planeRungeKutta4(Plane *plane, double ds)
{
  int numEqns{plane->numEqns};

  //  Allocate memory for the arrays.
  double* q = (double *)std::malloc(numEqns*sizeof(double));
  double* dq1 = (double *)std::malloc(numEqns*sizeof(double));
  double* dq2 = (double *)std::malloc(numEqns*sizeof(double));
  double* dq3 = (double *)std::malloc(numEqns*sizeof(double));
  double* dq4 = (double *)std::malloc(numEqns*sizeof(double));

  //  Retrieve the current values of the dependent
  //  and independent variables.
  double s{plane->s};
  for(int j=0; j<numEqns; ++j) {
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

  for(int j=0; j<numEqns; ++j) {
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
 
