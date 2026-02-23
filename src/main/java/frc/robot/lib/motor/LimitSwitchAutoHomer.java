package frc.robot.lib.motor;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import frc.robot.lib.motor.Motor.HomerType;

/**
 * Utility class for automatically homing a motor upon a limit switch depress.
 *
 * Note that this class mainly exists for symmetry reasons and is probably not
 * necessary.
 */
public class LimitSwitchAutoHomer {
    BooleanSupplier m_limitSwitchDepressedSupplier;
    Boolean m_invertedLimitSwitch;
    Double m_speed;
    Consumer<Double> m_setter;

    public LimitSwitchAutoHomer(BooleanSupplier limitSwitchDepressedSupplier, Boolean invertedLimitSwitch, Consumer<Double> setter, Double speed) {
        m_limitSwitchDepressedSupplier = limitSwitchDepressedSupplier;
        m_invertedLimitSwitch = invertedLimitSwitch;
        m_speed = speed;
    }

    public Boolean isHomed() {
        if (m_invertedLimitSwitch) {
            return !m_limitSwitchDepressedSupplier.getAsBoolean();
        }
        return m_limitSwitchDepressedSupplier.getAsBoolean();
    }

    /*
    * Should be called every tick.
    */
    public void update() {
        m_setter.accept(getOutput());
    }

    /*
    * Get the output of the homer to move towards the home position.
    */
   public Double getOutput() {
    if (isHomed()) {
        return 0d;
    } else {
        return m_speed;
    }
   }

   public HomerType asType() {
    return new HomerType._LimitSwitchAutoHomer(this);
   }
}
