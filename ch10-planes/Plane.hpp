
#ifndef __Plane_HPP__
#define __Plane_HPP__

//-----------------------------------------------------
// Structure: Plane
//-----------------------------------------------------
struct Plane
{
    // plane properties
    double wingArea{};
    double wingSpan{};
    double tailArea{};
    double clSlope0{};     // slope of Cl-alpha curve
    double cl0{};          // intercept of Cl-alpha curve
    double clSlope1{};     // post-stall slope of Cl-alpha curve
    double cl1{};          // post-stall intercept of Cl-alpha curve
    double alphaClMax{};   // alpha when Cl=Clmax
    double cdp{};          // parasite drag coefficient
    double eff{};          // induced drag efficiency coefficient
    double mass{};
    double enginePower{};
    double engineRps{};    // revolutions per second
    double propDiameter{};
    double a{};            //  propeller efficiency coefficient
    double b{};            //  propeller efficiency coefficient

    double bank{};
    double alpha{};        //  angle of attack
    double throttle{};
    int flap{};            //  flap deflection amount (0, 20, 40)
};

#endif


