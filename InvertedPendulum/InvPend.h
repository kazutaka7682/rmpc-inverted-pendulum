#pragma once

#include <cmath>
#include <iomanip>
#include <iostream>

class InvertedPendulum {
private:
    // Parameters
    const double dT; // control interval
    const double L;  // length of pendulum
    const double g;  // gravity
    const double m;  // pendulum mass
    const double M;  // car mass
    
    // States
    double angle;   // rad, pengulum angle
    double d_angle; // rad/s, pendulum speed
    double x;       // m, pos of car
    double d_x;     // m/s, vel of car

public:
    // constructor
    InvertedPendulum(double _dT, double _L, double _g, double _m, double _M, double _pos, double _angle, double _d_pos, double _d_angle)
      : dT(_dT), L(_L), g(_g), m(_m), M(_M) {
        x = _pos;           // [m]
        d_x = _d_pos;       // [m/s2]
        angle = _angle;     // [rad]
        d_angle = _d_angle; // [rad/s]
    }
    
    //compute state updation
    void UpdateState(double f) {
        double dd_x = 0.0;
        double dd_angle = 0.0;
        dd_x = (m*L*d_angle*d_angle*sin(angle) - m*g*cos(angle)*sin(angle) + f) / (m + M - m*cos(angle)*cos(angle));
        dd_angle = (m*L*d_angle*d_angle*sin(angle)*cos(angle) - m*g*sin(angle) - M*g*sin(angle) + f*cos(angle)) / (L*m*cos(angle)*cos(angle) - L*m - L*M);
        d_x = d_x + dT*dd_x;
        d_angle = d_angle + dT*dd_angle;
        x = x + dT*d_x;
        angle = angle + dT * d_angle;
    }
    
    // faster sin method
    double inline sinFast(double x) {
        return sin(x);
    }
    
    // faster cos method
    double inline cosFast(double x) {
        return cos(x);
    }
    
    // fast compute state updation
    void UpdateStateFast(double f) {
        double dd_x = 0.0;
        double dd_angle = 0.0;
        double sinAngle = sinFast(angle);
        double cosAngle = cosFast(angle);
        const double mL = m * L;
        const double mg = m * g;
        const double mPlusM= m + M;
        const double mPlusMg = mPlusM * g;
        const double ML = M * L;
        dd_x = ((mL*d_angle*d_angle - mg*cosAngle)*sinAngle+ f) / (mPlusM - m*cosAngle*cosAngle);
        dd_angle = (mL*d_angle*d_angle*sinAngle*cosAngle - mPlusMg*sinAngle + f*cosAngle) / (mL*cosAngle*cosAngle - mL - ML);
        d_x = d_x + dT*dd_x;
        d_angle = d_angle + dT*dd_angle;
        x = x + dT*d_x;
        angle = angle + dT * d_angle;
    }

    // example of state printing for debug
    void ShowState(double t) {
        std::cout << std::setprecision(6) << t << ":" << std::endl;
        std::cout << std::setprecision(3) << "a=" << angle << ", d_a=" << d_angle << ", x=" << x << ", d_x=" << d_x << std::endl;
    }
    
    // get angle
    double getAngle() {
        return angle;
    }
    
    // get derivative angle
    double getDerivativeAngle() {
        return d_angle;
    }
    
    // get dT
    double getdT() {
        return dT;
    }
};
