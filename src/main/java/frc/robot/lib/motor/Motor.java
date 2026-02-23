package frc.robot.lib.motor;

import java.util.Optional;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.PID;

/**
 * Class for abstracting different brands and types of motors to one unified
 * interface. Should be easy to expand.
 */
public class Motor {
    /**
     * The currently supported motors.
     */
    public enum SupportedMotorType {
        KRAKEN_X44,
        KRAKEN_X60,
        CTRE_Minion,
        REV_NEO,
        REV_NEO_2,
        REV_NEO_550,
        REV_Vortex;
    }

    private sealed interface ControlType permits ControlType._TalonFX, ControlType._SparkMax {
        record _TalonFX(TalonFX motor) implements ControlType {
            @Override
            public String toString() {
                return "TalonFX";
            }
        }

        record _SparkMax(SparkMax motor) implements ControlType {
            @Override
            public String toString() {
                return "SparkMax";
            }
        }
    }

    private ControlType m_motor;

    private ProfiledPIDController pidController;

    private Boolean PIDEnabled = false;

    /**
     * Construct a mostly brand agnostic motor controller.
     * 
     * @param CANid The id of the motor on the canbus.
     * @param type  The type of motor.
     */
    public Motor(int CANid, SupportedMotorType type) {
        switch (type) {
            case KRAKEN_X44:
            case KRAKEN_X60:
                m_motor = new ControlType._TalonFX(CTREHelpers.makeAndConfigureTalonFX(CANid));
                break;

            case CTRE_Minion:
            case REV_NEO:
            case REV_NEO_2:
            case REV_NEO_550:
            case REV_Vortex:
                m_motor = new ControlType._SparkMax(RevHelpers.makeSparkMaxMotorOfType(CANid, type));
                break;

            default:
                throw new RuntimeException(String.format("Unknown motor type '%s'", type.toString()));
        }
    }

    /**
     * Construct a mostly brand agnostic motor controller with PID control.
     * 
     * @param CANid The id of the motor on the canbus.
     * @param type  The type of motor.
     * @param pid   A {@link PID} control package.
     */
    public Motor(int CANid, SupportedMotorType type, PID pid) {
        switch (type) {
            case KRAKEN_X44:
            case KRAKEN_X60:
                m_motor = new ControlType._TalonFX(CTREHelpers.makeAndConfigureTalonFX(CANid));
                break;

            case CTRE_Minion:
            case REV_NEO:
            case REV_NEO_2:
            case REV_NEO_550:
            case REV_Vortex:
                m_motor = new ControlType._SparkMax(RevHelpers.makeSparkMaxMotorOfType(CANid, type));
                break;

            default:
                throw new RuntimeException(String.format("Unknown motor type '%s'", type.toString()));
        }

        initializePID(pid);
    }

    /**
     * Get the CAD id of the device.
     * 
     * @return The CAN id of the device.
     */
    public int getID() {
        if (m_motor instanceof ControlType._TalonFX) {
            TalonFX fx = ((ControlType._TalonFX) m_motor).motor();

            return fx.getDeviceID();

        } else {
            SparkMax spark = ((ControlType._SparkMax) m_motor).motor();

            return spark.getDeviceId();
        }
    }

    /**
     * Set the percent output of the motor. Range (-1 - 1)
     * 
     * @param speed Speed to set between -1 and 1.
     */
    public void set(double speed) {
        if (m_motor instanceof ControlType._TalonFX) {
            TalonFX fx = ((ControlType._TalonFX) m_motor).motor();

            fx.set(speed);

        } else {
            SparkMax spark = ((ControlType._SparkMax) m_motor).motor();

            spark.set(speed);
        }
    }

    /**
     * Get the angle the motor is at relative from where it was turned on.
     * To get the absolute position, look in to CAN encoders or encoders on the
     * SparkMax.
     * 
     * @return Position in degrees, wrapping 0 - 360.
     */
    public Double getRelativePosition() {
        if (m_motor instanceof ControlType._TalonFX) {
            TalonFX fx = ((ControlType._TalonFX) m_motor).motor();

            // convert rotations to degrees.
            return absoluteRotation(fx.getPosition().getValueAsDouble() * 360);

        } else {
            SparkMax spark = ((ControlType._SparkMax) m_motor).motor();

            // convert rotations to degrees.
            return absoluteRotation(spark.getAlternateEncoder().getPosition() * 360);
        }
    }

    /**
     * Try to get the absolute position of the motor. This depends on if an external
     * encoder has been attached.
     * 
     * @return A optional containing the position if it could be read, or an empty
     *         optional otherwise.
     */
    public Optional<Double> getAbsolutePosition() {
        if (m_motor instanceof ControlType._TalonFX) {
            // Can't get the absolute position of a TalonFX without a separate encoder.
            return Optional.empty();
        } else {
            SparkMax spark = ((ControlType._SparkMax) m_motor).motor();
            AbsoluteEncoder encoder;

            try {
                encoder = spark.getAbsoluteEncoder();
            } catch (IllegalStateException e) {
                // no encoder
                return Optional.empty();
            }

            return Optional.of(encoder.getPosition());
        }
    }

    /**
     * Set the output of one motor to mirror another.
     * 
     * @param motor   The motor to follow
     * @param opposed Whether to flip the movement of the motor.
     */
    public void setFollowingOther(Motor motor, boolean opposed) {
        if (motor.getType() != this.getType()) {
            DriverStation
                    // #racialprofiling
                    .reportError(String.format(
                            "Can't follow a motor of a different type! (I am a %s, they are a %s) (if you reallllly need to you probably can, but it is not officially supported and might not work",
                            m_motor.toString(), motor.m_motor.toString()), true);
            return;
        }

        if (m_motor instanceof ControlType._TalonFX) {
            TalonFX fx = ((ControlType._TalonFX) m_motor).motor();

            fx.setControl(
                    new Follower(motor.getID(), opposed ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
        } else {
            SparkMax spark = ((ControlType._SparkMax) m_motor).motor();

            // reconfiguring does not clear parameters, so it's fine.
            spark.configure(new SparkMaxConfig().follow(motor.getID(), opposed), ResetMode.kNoResetSafeParameters,
                    PersistMode.kPersistParameters);
        }
    }

    /**
     * Function to reset the measured position of the motor to zero. Useful if you
     * just got to a position where you know you must be at some angle, aka homing.
     */
    public void setAsHomed() {
        if (m_motor instanceof ControlType._TalonFX) {
            TalonFX fx = ((ControlType._TalonFX) m_motor).motor();

            // set the relative encoder's position to zero.
            fx.setPosition(0);
        } else {
            SparkMax spark = ((ControlType._SparkMax) m_motor).motor();

            // set the relative encoder's position to zero.
            spark.getAlternateEncoder().setPosition(0);
        }

        if (pidController != null) {
            // reset the PID controller's setpoint.
            pidController.reset(0);
        }
    }

    /**
     * Setup PID control for this motor.
     * Zeros the PID controller to the current position.
     * 
     * @param pid The {@link PID} control package.
     */
    public void initializePID(PID pid) {
        pidController = pid.getController();
        pidController.reset(getRelativePosition());
    }

    /**
     * Set the PID controls goal to position
     * 
     * @param position The new goal.
     */
    public void setPIDGoal(double position) {
        pidController.setGoal(position);
    }

    /**
     * Get the underling {@link ProfiledPIDController}.
     * 
     * @return The controller.
     */
    public ProfiledPIDController getPIDController() {
        return pidController;
    }

    /**
     * Update the motor's position using PID control. Should be called every
     * periodic tick.
     */
    public void updatePID() {
        if (PIDEnabled) {
            set(pidController.calculate(getRelativePosition()));
        }
    }

    /**
     * Set whether PID control should be enabled. Starts disabled by default.
     * 
     * @param enabled Should we do PID control?
     */
    public void setPIDEnabled(Boolean enabled) {
        PIDEnabled = enabled;

        pidController.reset(getRelativePosition());
    }

    /**
     * Gets the type of the motor.
     * 
     * @return The type of the motor.
     */
    public ControlType getType() {
        return m_motor;
    }

    /**
     * Normalizes a rotational input value to the range [0, 360) degrees.
     *
     * @param input The input rotational value.
     * @return The normalized rotational value within the range [0, 360) degrees.
     */
    public double absoluteRotation(double input) {
        return (input % 360 + 360) % 360;
    }
}
