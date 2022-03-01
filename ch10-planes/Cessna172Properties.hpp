
#ifndef __Cessna172Properties_HPP__
#define __Cessna172Properties_HPP__

//-----------------------------------------------------
// Structure: Cessna172Properties
// Description: Cessna 172 coeficients and parameters used
//              to calculate forces
//-----------------------------------------------------
struct Cessna172Properties
{
    // lift coeficients
    double wingArea{16.2};         // wing wetted area, m^2
    double wingSpan{10.9};         // wing span, m
    double tailArea{2.0};          // tail wetted area, m^2
    double clSlope0{0.0889};       // slope of Cl-alpha curve
    double cl0{0.178};             // intercept of post-stall Cl-alpha curve
    double clSlope1{-0.1};         // slope of post-stall Cl-alpha curve
    double cl1{3.2};               // intercept of post-stall Cl-alpha curve
    double alphaClMax{16.0};       // alpha at Cl(max)
    // drag coefficients
    double cdp{0.034};             // parasite drag coefficient
    double eff{0.77};              // induced drag efficiency coefficient
    // gravity
    double mass{1114.0};           // airplane mass, kg
    // thrust
    double enginePower{119310.0};  // peak engine power, W
    double engineRps{40.0};        // engine turnover rate, rev/s
    double propDiameter{1.905};    // propeller diameter, m
    double a{1.83};                // propeller efficiency curve fit coefficient
    double b{-1.32};               // propeller efficiency curve fit coefficient
};

#endif
