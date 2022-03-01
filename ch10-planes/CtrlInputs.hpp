#ifndef __CtrlInputs_HPP__
#define __CtrlInputs_HPP__

//-----------------------------------------------------
// Structure: CtrlInputs
// Description: Control inputs from user
//-----------------------------------------------------
struct CtrlInputs
{
    double bank{};              // bank angle
    double alpha{4.0};          // angle of attack
    double throttle{1.0};       // throttle (0.0->1.0)
    int flap{};                 // flap deflection amount (0, 20, 40)
};

#endif
