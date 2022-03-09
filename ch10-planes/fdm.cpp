
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <memory>
#include <tuple>

#include "Cessna172Properties.hpp"
#include "CtrlInputs.hpp"
#include "Rk4Data.hpp"

const double pi{std::acos(-1.0)};
const double G{-9.81};

std::tuple<double, double, double>
calculate_forces(const Cessna172Properties& prop, const CtrlInputs& inputs, const double altitude, const double velocity)
{
    // air density
    const double temperature{288.15 - 0.0065 * altitude};
    const double grp{(1.0 - 0.0065 * altitude / 288.15)};
    const double pressure{101325.0 * std::pow(grp, 5.25)};
    const double density{0.00348 * pressure / temperature};

    // power drop-off factor
    const double omega{density / 1.225};
    const double factor{(omega - 0.12) / 0.88};

    // compute thrust
    const double advanceRatio{velocity / (prop.engineRps * prop.propDiameter)};
    const double a{prop.a};
    const double b{prop.b};
    const double thrust{inputs.throttle * factor * prop.enginePower * (a + b * advanceRatio * advanceRatio) / (prop.engineRps * prop.propDiameter)};

    // compute lift coefficient - the Cl curve is modeled using two straight lines
    double cl{inputs.alpha < prop.alphaClMax ? prop.clSlope0 * inputs.alpha + prop.cl0 : prop.clSlope1 * inputs.alpha + prop.cl1};

    // include the effect of flaps
    if (inputs.flap == 20) {
        cl += 0.25;
    }
    if (inputs.flap == 40) {
        cl += 0.5;
    }
    // include ground effects if the plane is within 5 meters of the ground
    if (altitude < 5.0) {
        cl += 0.25;
    }

    // compute lift
    const double wingArea{prop.wingArea};
    const double lift{0.5 * cl * density * velocity * velocity * wingArea};

    // compute drag coefficient
    const double wingSpan{prop.wingSpan};
    const double aspectRatio{wingSpan * wingSpan / wingArea};
    const double cd{prop.cdp + cl * cl / (pi * aspectRatio * prop.eff)};

    // compute drag force
    const double drag{0.5 * cd * density * velocity * velocity * wingArea};

    return std::make_tuple(thrust, lift, drag);
}

//-----------------------------------------------------
// loads the right-hand sides for the plane ODEs
//-----------------------------------------------------
void plane_rhs(const Cessna172Properties& prop,
               const CtrlInputs& inputs,
               const double* const q, const double* const deltaQ,
               const double dt, const double qScale,
               double* dq)
{
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
    const double z{newQ[5]};
    const double vh{std::sqrt(vx * vx + vy * vy)};
    const double velocity{std::sqrt(vx * vx + vy * vy + vz * vz)};

    const std::tuple<double, double, double> forces = calculate_forces(prop, inputs, z, velocity);
    const double thrust{std::get<0>(forces)};
    const double lift{std::get<1>(forces)};
    const double drag{std::get<2>(forces)};

    // convert bank angle from degrees to radians
    // angle of attack is not converted because the
    // Cl-alpha curve is defined in terms of degrees
    const double bank{inputs.bank * pi / 180.0};

    // define some shorthand convenience variables for use with the rotation matrix
    // compute the sine and cosines of the climb angle, bank angle, and heading angle

    const double cosW{std::cos(bank)};  // bank angle
    const double sinW{std::sin(bank)};

    const double cosP{velocity == 0.0 ? 1.0: vh/velocity};   // climb angle
    const double sinP{velocity == 0.0 ? 0.0: vz/velocity};

    const double cosT{vh == 0.0 ? 1.0: vx/vh};   // heading angle
    const double sinT{vh == 0.0 ? 0.0: vy/vh};

    // convert the thrust, drag, and lift forces into x-, y-, and z-components using the rotation matrix
    const double Fx{cosT * cosP * (thrust - drag) + (sinT * sinW - cosT * sinP * cosW) * lift};
    const double Fy{sinT * cosP * (thrust - drag) + (-cosT * sinW - sinT * sinP * cosW) * lift};
    double Fz{sinP * (thrust - drag) + cosP * cosW * lift};

    // add the gravity force to the z-direction force
    Fz += prop.mass * G;

    // since the plane can't sink into the ground, if the altitude is less than or equal to zero and the z-component
    // of force is less than zero, set the z-force to be zero
    if (z <= 0.0 && Fz <= 0.0) {
        Fz = 0.0;
    }

    // load the right-hand sides of the ODE's
    dq[0] = dt * (Fx / prop.mass);
    dq[1] = dt * vx;
    dq[2] = dt * (Fy / prop.mass);
    dq[3] = dt * vy;
    dq[4] = dt * (Fz / prop.mass);
    dq[5] = dt * vz;

    return;
}

//-----------------------------------------------------
// 4th-order Runge-Kutta solver for plane motion
//-----------------------------------------------------
void eom_rk4(const Cessna172Properties& plane, const CtrlInputs& ctrl_inputs, Rk4Data* rk4_data, const double dt)
{
    const int numEqns{rk4_data->numEqns};

    // allocate memory for the arrays
    auto q = new double[numEqns];
    auto dq1 = new double[numEqns];
    auto dq2 = new double[numEqns];
    auto dq3 = new double[numEqns];
    auto dq4 = new double[numEqns];

    // retrieve the current values of the dependent
    // and independent variables
    for (int j{}; j < numEqns; ++j) {
        q[j] = rk4_data->q[j];
    }

    // compute the four Runge-Kutta steps, then return 
    // value of planeRightHandSide method is an array
    // of delta-q values for each of the four steps
    plane_rhs(plane, ctrl_inputs, q, q,   dt, 0.0, dq1);
    plane_rhs(plane, ctrl_inputs, q, dq1, dt, 0.5, dq2);
    plane_rhs(plane, ctrl_inputs, q, dq2, dt, 0.5, dq3);
    plane_rhs(plane, ctrl_inputs, q, dq3, dt, 1.0, dq4);

    // update the dependent and independent variable values
    // at the new dependent variable location and store the
    // values in the ODE object arrays
    for (int j{}; j < numEqns; ++j) {
        q[j] = q[j] + (dq1[j] + 2.0*dq2[j] + 2.0*dq3[j] + dq4[j])/6.0;
        rk4_data->q[j] = q[j];
    }

    // free memory
    delete[] q;
    delete[] dq1;
    delete[] dq2;
    delete[] dq3;
    delete[] dq4;

    return;
}

