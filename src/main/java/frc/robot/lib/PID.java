package frc.robot.lib;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;

public class PID {
    double m_P = 0;
    double m_I = 0;
    double m_D = 0;

    double m_t = 0.2;

    Constraints m_constraints = new Constraints(0, 0);

    ProfiledPIDController m_controller = null;

    public PID(double kP, double kI, double kD, Constraints constraints) {
        m_constraints = constraints;

        m_P = kP;
        m_D = kD;
        m_I = kI;

        buildController();
    }

    public PID(double kP, double kI, double kD) {
        m_P = kP;
        m_D = kD;
        m_I = kI;

        buildController();
    }

    public PID(double kP, double kI, double kD, double t, Constraints constraints) {
        m_constraints = constraints;

        m_P = kP;
        m_D = kD;
        m_I = kI;

        m_t = t;

        buildController();
    }

    public PID(double kP, double kI, double kD, double t) {
        m_P = kP;
        m_D = kD;
        m_I = kI;

        m_t = t;

        buildController();
    }

    public void setConstraints(Constraints constraints) {
        m_constraints = constraints;
        buildController();
    }

    public ProfiledPIDController getController() {
        buildController();

        if (m_constraints.equals(new Constraints(0, 0))) {
            DriverStation.reportWarning("You probably want to set the constraints!", true);
        }

        return m_controller;
    }

    public void buildController() {
        assert (m_P >= 0 && m_I >= 0 && m_D >= 0 && m_t >= 0);

        m_controller = new ProfiledPIDController(m_P, m_I, m_D, m_constraints, m_t);
    }
}
