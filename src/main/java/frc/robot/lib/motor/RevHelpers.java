package frc.robot.lib.motor;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.Constants;
import frc.robot.lib.motor.Motor.SupportedMotorType;


public class RevHelpers {
    /**
     * Check if a given RevLibError is OK.
     * If not, throw an error.
     * 
     * @return Whether is error is OK or not.
     */
    public static boolean checkRevLibOk(REVLibError error) {
        if (error != REVLibError.kOk) {
            DriverStation.reportError(String.format("Rev Lib returned an error! %s", error.toString()), true);
            return false;
        }

        return true;
    }

    /**
     * Construct a motor that uses a SparkMax motor controller.
     */
    public static SparkMax makeSparkMaxMotor(int id, SparkBaseConfig config, MotorType type) {
        SparkMax motor = new SparkMax(id, type);

        REVLibError error = motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        checkRevLibOk(error);

        return motor;
    }

    /**
     * Make a new SparkMax controlled motor with a set CAN id and type.
     * 
     * @param id   The CAN id of the SparkMax.
     * @param type The type of SparkMax capable motor.
     * 
     * @return The new motor object.
     */
    public static SparkMax makeSparkMaxMotorOfType(int id, SupportedMotorType type) {
        switch (type) {
            case REV_NEO:
                return makeSparkMaxMotor(id, Constants.MotorConfigs.NEO, MotorType.kBrushless);

            case REV_NEO_2:
                return makeSparkMaxMotor(id, Constants.MotorConfigs.NEO_2, MotorType.kBrushless);

            case REV_NEO_550:
                return makeSparkMaxMotor(id, Constants.MotorConfigs.NEO_550, MotorType.kBrushless);

            case REV_Vortex:
                return makeSparkMaxMotor(id, Constants.MotorConfigs.VORTEX, MotorType.kBrushless);

            case CTRE_Minion:
                return makeSparkMaxMotor(id, Constants.MotorConfigs.CTRE_MINION, MotorType.kBrushless);

            default:
                throw new RuntimeException(String.format("Unknown SparkMax motor type '%s'", type.toString()));
        }
    }
}
