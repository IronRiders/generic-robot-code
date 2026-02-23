package frc.robot.lib.motor;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.Constants;

public class CTREHelpers {
    /**
     * Check if a given status code is OK.
     * If not, throw a warning or error as appropriate.
     * 
     * @return Whether is code is OK or not.
     */
    public static boolean checkStatusCodeOk(StatusCode code) {
        if (code.isError()) {
            DriverStation.reportError(
                    String.format("CTRE Status code returned an error! %s: $s", code.getName(), code.getDescription()),
                    true);
        }

        if (code.isWarning()) {
            DriverStation.reportWarning(String.format("CTRE Status code returned an warning! %s: $s", code.getName(),
                    code.getDescription()), true);
        }

        return code.isOK();
    }

    /**
     * Make a TalonFX and configure it with the default config.
     * 
     * @param id The CAD id of this motor.
     * 
     * @return The new motor.
     */
    public static TalonFX makeAndConfigureTalonFX(int id) {
        TalonFX motor = new TalonFX(id);

        StatusCode code = motor.getConfigurator().apply(Constants.MotorConfigs.KRAKEN_CONFIGURATION);

        checkStatusCodeOk(code);

        return motor;
    }

    /**
     * Make a TalonFX and configure it with the provided config.
     * 
     * @param id            The CAD id of this motor.
     * @param configuration The {@link TalonFXConfiguration} that should be applied.
     * 
     * @return The new motor.
     */
    public static TalonFX makeTalonFX(int id, TalonFXConfiguration configuration) {
        TalonFX motor = new TalonFX(id);

        StatusCode code = motor.getConfigurator().apply(configuration);

        checkStatusCodeOk(code);

        return motor;
    }
}
