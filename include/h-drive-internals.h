struct MotorSettings {
  double max_accel = 1; //rpm / 20ms
  double max_velo = 200; //rpm
  double lock_p = 5; // rpm / rev
  double lock_ff = 5; // rpm
  MotorSettings(double a, double v, double p, double ff) {
    max_accel = a;
    max_velo = v;
    lock_p = p;
    lock_ff = ff;
  };
  MotorSettings();
};

MotorSettings DRIVE_MOTOR_SETTINGS(1,100,5,5);
MotorSettings INTAKE_MOTOR_SETTINGS(200,200,0,0);

class Motor {
  public:
    Motor();
    Motor(vex::motor, MotorSettings);
    void drive(double); //rpm
    double getPosition(); //rev
    void loop();
    void toggleLock();
    void setLockPos(double);
  private:
    MotorSettings m_settings;
    vex::motor* m_motor;
    double lastVelocity = 0; //rpm
    double targetVelocity = 0; //rpm
    bool isLocked = false;
    double lockedPosition = 0; //rpm
};

class HDrive {
  public:
    static constexpr double ROTATION_DIAMETER = 5; //in
    static constexpr double WHEEL_DIAMETER = 4; //in
    HDrive(Motor, Motor, Motor);
    void setDriveVelocities(double, double, double);
    void setWheelVelocities(double, double, double);
    void loop();
  private:
    Motor m_left;
    Motor m_right;
    Motor m_strafe;
};