package frc.robot.lib;

import java.io.File;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig.Presets;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Filesystem;

public class Constants {
    /**
     * Length and width of the field, measurements in inches.
     * Also contains the center point as a Pose2d
     * TODO: Update to <current year>. Last updated 2026.
    */
    public static class Field {
        public static final double FIELD_LENGTH = 651.22;
        public static final double FIELD_WIDTH = 317.69;
        public static final Pose2d CENTER_INCHES = new Pose2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2, new Rotation2d());
        // Convert inches to meters by multiplying by 0.0254.
        public static final Pose2d CENTER = new Pose2d(CENTER_INCHES.getTranslation().times(0.0254), CENTER_INCHES.getRotation());
    }

    public static class MotorConfigs {
        public static final TalonFXConfiguration KRAKEN_CONFIGURATION = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(40));

        public static final SparkBaseConfig NEO = Presets.REV_NEO;
        public static final SparkBaseConfig NEO_2 = Presets.REV_NEO_2;
        public static final SparkBaseConfig NEO_550 = Presets.REV_NEO_550;
        public static final SparkBaseConfig VORTEX = Presets.REV_Vortex;
        public static final SparkBaseConfig CTRE_MINION = Presets.CTRE_Minion;

    }

    public static class Identifiers {
        // TODO: Repace all ids with correct values.

        // Remember to also update the swervedrive.json file with the new id.
        public static final int GYROSCOPE = 1;
        

        // You can get this info by opening DriverStation and looking under the devices tab.
        public static final int PRIMARY_CONTROLLER_PORT = 0;
    }

    public static class Drive {
        // Find the deploy directory once we're actually on the robot and then get the
        // "swerve" folder.
        public static final File SWERVE_JSON_DIRECTORY = new File(Filesystem.getDeployDirectory(), "swerve");

        // The maximum speed the swerve drive is allowed to go.
        // We use a LinearVelocity to make it clear what units this is in, but just a
        // double works too.
        public static final LinearVelocity MAX_SPEED = Units.MetersPerSecond.of(3);
        public static final LinearAcceleration MAX_ACCELERATION = Units.MetersPerSecondPerSecond.of(3);


        // Maximum rotation the swerve drive is allowed to do. !This is not strictly enforced!
        public static final AngularVelocity MAX_ROTATION = Units.RadiansPerSecond.of(Math.PI);
        public static final AngularAcceleration MAX_ROTATION_ACCELERATION = Units.RadiansPerSecondPerSecond.of(Math.PI);



        // Configure the PID controller that the drivetrain uses to maintain a certain position.
        public static final PPHolonomicDriveController SWERVE_CONFIG = new PPHolonomicDriveController(
                new PIDConstants(10.0, 0.0, 0.0), // Translation PID
                new PIDConstants(10.0, 0.0, 0.0) // Rotation PID
        );

        // How much does the driver have to be pushing on the control stick to cancel pathfinding.
        public static final Double CONTROL_OVERRIDE_THRESHOLD = 0.3; // (0-1)

        public static final PathConstraints PATHFIND_CONSTRAINTS = new PathConstraints(MAX_SPEED,
                MAX_ACCELERATION, MAX_ROTATION, MAX_ROTATION_ACCELERATION);
    }
}
