#include "vex.h"
#include <cmath>
#include <algorithm>

using namespace std;

Motor::Motor(vex::motor m, MotorSettings settings) {
  m_settings = settings;
  m_motor = &m;
  m_motor->setStopping(brakeType::hold);
}

/* motor loop, designed to run on 20ms cycle */
void Motor::loop() {
  //motor position lock
  if (isLocked) {
    //motor lock PID
    double err = getPosition() - lockedPosition;
    double feed_fwd;
    if (err > 0) {
      feed_fwd = -m_settings.lock_ff;
    } else {
      feed_fwd = m_settings.lock_ff;
    }

    lastVelocity = 0;

    m_motor->spin(directionType::fwd, m_settings.lock_p * err + feed_fwd, velocityUnits::rpm);
  } else {
    //velocity acceleration limiting
    if (targetVelocity > lastVelocity) {
      lastVelocity = min(targetVelocity, lastVelocity + m_settings.max_accel);
    } else if (targetVelocity < lastVelocity) {
      lastVelocity = max(targetVelocity, lastVelocity - m_settings.max_accel);
    }
    m_motor->spin(directionType::fwd, lastVelocity, velocityUnits::rpm);
  }
}

double Motor::getPosition() {
  return m_motor->position(rotationUnits::rev);
}

void Motor::drive(double v) { //rpm
  targetVelocity = v; //TODO: limit velocity
}

void Motor::toggleLock() {
  isLocked = !isLocked;
  lockedPosition = getPosition();
}

void Motor::setLockPos(double p) {
  lockedPosition = p;
}

HDrive::HDrive(Motor left, Motor right, Motor strafe) {
  m_left = left;
  m_right = right;
  m_strafe = strafe;
}

/* vx is to the front, vy is to the left ( units in/s ).
   omega is ccw about the center (units rot/s )  */
void HDrive::setDriveVelocities(double v_x, double v_y, double omega) {
  double v_rot = omega * M_PI * ROTATION_DIAMETER; //in/s
  double v_r = v_x + v_rot; //in/s
  double v_l = v_x - v_rot; //in/s

  m_left.drive(v_l / 60 * (M_PI * WHEEL_DIAMETER));
  m_right.drive(v_r / 60 * (M_PI * WHEEL_DIAMETER));
  m_strafe.drive(v_y / 60 * (M_PI * WHEEL_DIAMETER));
}

//all velocities in/s
void HDrive::setWheelVelocities(double v_l, double v_r, double v_s) {
  m_left.drive(v_l / 60 * (M_PI * WHEEL_DIAMETER));
  m_right.drive(v_r / 60 * (M_PI * WHEEL_DIAMETER));
  m_strafe.drive(v_s / 60 * (M_PI * WHEEL_DIAMETER));
}

void HDrive::loop() {
  m_left.loop();
  m_right.loop();
  m_strafe.loop();
}