package frc.robot.lib.motor;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.stream.Collectors;

import frc.robot.lib.motor.Motor.HomerType;

/**
* Utility class for automatically homing a motor upon a voltage spike.
*/
public class CurrentAutoHomer {
    final int SAMPLES = 20;
    final Double SPIKE_THRESHOLD = 20d;

    DoubleSupplier m_motorCurrentSupplier;
    List<Double> m_averages = new ArrayList<Double>(SAMPLES);
    Double m_speed;
    Consumer<Double> m_setter;

    public CurrentAutoHomer(DoubleSupplier motorCurrentSupplier, Consumer<Double> setter, Double speed) {
        m_motorCurrentSupplier = motorCurrentSupplier;
        m_speed = speed;
        m_setter = setter;
    }
    
    /*
    * Should be called every tick.
    */
    public void update() {
        m_averages.add(m_motorCurrentSupplier.getAsDouble());

        while (m_averages.size() >= SAMPLES) {
            m_averages.remove(SAMPLES);
        }

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

    /*
    * We can suppose that the motor's torque current will follow a similar pattern to the below graph, where the x axis is time, and the y is voltage.
    * The start of the graph shows the startup torque, the current required to move the mechanism from it's starting position and get it up to speed.
    * Then we see the moving torque, or the voltage required to keep the mechanism at speed while moving towards the hardstop.
    * Finally, we see the stopped torque, where the motor is fighting to keep moving event though it is at the hardstop.
    * This prolonged current spike is what we want to detect.
    *
    * ████████████████████████████████████
    * █                      Stopped \   █
    * █                         ┌────────█
    * █ Starting \              │        █
    * █       ┌─────┐          ┌┘        █
    * █      ┌┘     └┐         │         █
    * █     ┌┘       └┐        │         █
    * █    ┌┘         └────────┘         █
    * █   ┌┘     Moving ^                █
    * █───┘                              █
    * ████████████████████████████████████
    */

    public Boolean isHomed() {
        return Math.abs(m_motorCurrentSupplier.getAsDouble() - getRollingAverage()) > SPIKE_THRESHOLD;
    }

    private Double getRollingAverage() {
        return m_averages.stream().collect(Collectors.averagingDouble(d -> d.doubleValue()));
    }

    public HomerType asType() {
    return new HomerType._CurrentAutoHomer(this);
   }
}
