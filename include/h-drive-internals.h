class HDrive {
  public:
    static constexpr double ROTATION_DIAMETER = 5; //in
    static constexpr double WHEEL_DIAMETER = 4; //in
    static constexpr double A_X = .6;
    static constexpr double A_Y = 1.2;
    static constexpr double A_OMEGA = 1.2;
    HDrive(vex::motor, vex::motor, vex::motor);
    void setDriveVelocities(double, double, double);
    void setWheelVelocities(double, double, double);
    void loop();
  private:
    vex::motor* m_left;
    vex::motor* m_right;
    vex::motor* m_strafe;
    double v_x_last = 0;
    double v_y_last = 0;
    double omega_last = 0;
};