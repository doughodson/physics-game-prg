
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <memory>

#include "plane.hpp"

const double pi{std::acos(-1.0)};
const double G{-9.81};

//-----------------------------------------------------
// loads the right-hand sides for the plane ODEs
//-----------------------------------------------------
void plane_rhs(const Plane* plane, 
               const double* const q, const double* const deltaQ,
               const double dt, const double qScale,
               double* dq)
{
    // convenience variables
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
    for (int i{}; i < 6; ++i) {
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
    double cl{alpha < alphaClMax ? clSlope0 * alpha + cl0 : clSlope1 * alpha + cl1};

    // include effects of flaps and ground effects
    // -- ground effects are present if the plane is within 5 meters of the ground
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

    const double cosW{std::cos(bank)};  // bank angle
    const double sinW{std::sin(bank)};

    const double cosP{vtotal == 0.0 ? 1.0: vh/vtotal};   // climb angle
    const double sinP{vtotal == 0.0 ? 0.0: vz/vtotal};

    const double cosT{vh == 0.0 ? 1.0: vx/vh};   // heading angle
    const double sinT{vh == 0.0 ? 0.0: vy/vh};

    // convert the thrust, drag, and lift forces into x-, y-, and z-components using the rotation matrix
    const double Fx{cosT * cosP * (thrust - drag) + (sinT * sinW - cosT * sinP * cosW) * lift};
    const double Fy{sinT * cosP * (thrust - drag) + (-cosT * sinW - sinT * sinP * cosW) * lift};
    double Fz{sinP * (thrust - drag) + cosP * cosW * lift};

    // add the gravity force to the z-direction force
    Fz += mass * G;

    // since the plane can't sink into the ground, if the altitude is less than or equal to zero and the z-component
    // of force is less than zero, set the z-force to be zero
    if (z <= 0.0 && Fz <= 0.0) {
        Fz = 0.0;
    }

    // load the right-hand sides of the ODE's
    dq[0] = dt * (Fx / mass);
    dq[1] = dt * vx;
    dq[2] = dt * (Fy / mass);
    dq[3] = dt * vy;
    dq[4] = dt * (Fz / mass);
    dq[5] = dt * vz;

    return;
}

//-----------------------------------------------------
// 4th-order Runge-Kutta solver for plane motion
//-----------------------------------------------------
void eom(Plane *plane, const double dt)
{
    int numEqns{plane->numEqns};

    // allocate memory for the arrays
    auto q = new double[numEqns];
    auto dq1 = new double[numEqns];
    auto dq2 = new double[numEqns];
    auto dq3 = new double[numEqns];
    auto dq4 = new double[numEqns];

    // retrieve the current values of the dependent
    // and independent variables
    for (int j=0; j<numEqns; ++j) {
        q[j] = plane->q[j];
    }

    // compute the four Runge-Kutta steps, then return 
    // value of planeRightHandSide method is an array
    // of delta-q values for each of the four steps
    plane_rhs(plane, q, q,   dt, 0.0, dq1);
    plane_rhs(plane, q, dq1, dt, 0.5, dq2);
    plane_rhs(plane, q, dq2, dt, 0.5, dq3);
    plane_rhs(plane, q, dq3, dt, 1.0, dq4);

    // update simulation time
    plane->time += dt;

    // update the dependent and independent variable values
    // at the new dependent variable location and store the
    // values in the ODE object arrays
    for(int j=0; j<numEqns; ++j) {
        q[j] = q[j] + (dq1[j] + 2.0*dq2[j] + 2.0*dq3[j] + dq4[j])/6.0;
        plane->q[j] = q[j];
    }

    // free memory
    delete[] q;
    delete[] dq1;
    delete[] dq2;
    delete[] dq3;
    delete[] dq4;

    return;
}
 
