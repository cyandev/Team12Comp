#include "vex.h"
#include <cmath>
#include <algorithm>
#include <iostream>

using namespace std;

HDrive::HDrive(vex::motor left, vex::motor right, vex::motor strafe) {
  m_left = &left;
  m_right = &right;
  m_strafe = &strafe;
}

/* vx is to the front, vy is to the left ( units in/s ).
   omega is ccw about the center (units rot/s )  */
void HDrive::setDriveVelocities(double v_x, double v_y, double omega) {
  //acceleration limiting
  if (v_x > v_x_last) 
    v_x_last = min(v_x, v_x_last + A_X);
  if (v_x < v_x_last) 
    v_x_last = max(v_x, v_x_last - A_X);
  
  if (v_y > v_y_last) 
    v_y_last = min(v_y, v_y_last + A_Y);
  if (v_y < v_y_last) 
    v_y_last = max(v_y, v_y_last - A_Y);

  if (omega > omega_last) 
    omega_last = min(omega, omega_last + A_OMEGA);
  if (omega < omega_last) 
    omega_last = max(omega, omega_last - A_OMEGA);

  // now v_{}_last is the current velocity

  double v_rot = omega_last * M_PI * ROTATION_DIAMETER; //in/s
  double v_r = v_x_last - v_rot; //in/s
  double v_l = v_x_last + v_rot; //in/s

  
  m_left->spin(directionType::fwd, v_l * 60 / (M_PI * WHEEL_DIAMETER) * 5, velocityUnits::rpm);
  m_right->spin(directionType::fwd, v_r * 60 / (M_PI * WHEEL_DIAMETER) * 5, velocityUnits::rpm);
  m_strafe->spin(directionType::fwd, v_y * 60 / (M_PI * WHEEL_DIAMETER), velocityUnits::rpm);
}

//all velocities in/s
void HDrive::setWheelVelocities(double v_l, double v_r, double v_s) {
  m_left->spin(directionType::fwd, v_l * 60 / (M_PI * WHEEL_DIAMETER) * 5, velocityUnits::rpm);
  m_right->spin(directionType::fwd, v_r * 60 / (M_PI * WHEEL_DIAMETER) * 5, velocityUnits::rpm);
  m_strafe->spin(directionType::fwd, v_s * 60 / (M_PI * WHEEL_DIAMETER) * 5, velocityUnits::rpm);
}