package frc.robot.drive.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.io.IOException;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.core.RobotContainer;
import frc.robot.drive.PathPlannerHelpers;
import frc.robot.lib.Constants;
import frc.robot.lib.SubsystemHelper;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

/*
 * Example SwerveDrive control subsystem using the YAGSL library.
 * 
 * This keeps track of the robot's position and angle, and uses the controller input to figure
 * out how the individual modules need to turn and be angled.
 */
public class SwerveDriveSubsystem extends SubsystemHelper {
    private static SwerveDriveCommands commands;

    // Store our SwerveDrive.
    private static SwerveDrive swerveDrive;

    private static boolean rotationInverted = false;
    private static boolean driveInverted = false;

    // Construct a instance of our gyroscope / IMU, in this case a CTRE Pigeon2.
    // Update accordingly to your robot.
    // Remember to also update the swervedrive.json file with the new info.
    private static Pigeon2 gyroscope = new Pigeon2(Constants.Identifiers.GYROSCOPE);

    /*
     * Constructor. We:
     * Create the Swerve Drive,
     * Setup PathPlanner for use in auto and pathfinding.
     */
    public SwerveDriveSubsystem() throws RuntimeException {
        // Construct a new commands instance
        commands = new SwerveDriveCommands(this);

        // Create the SwerveDrive object.
        try {
            swerveDrive = new SwerveParser(Constants.Drive.SWERVE_JSON_DIRECTORY) // YAGSL reads from deploy/swerve.
                    .createSwerveDrive(Constants.Drive.MAX_SPEED.in(MetersPerSecond));
        } catch (IOException e) { // Instancing the SwerveDrive can throw an error, so we need to catch that.
            throw new RuntimeException("Error configuring swerve drive!", e);
        }

        // This will try to yank the robot to face straight while moving. we don't like
        // it, but you should experiment!
        // See https://docs.yagsl.com/overview/our-features/heading-correction for
        // details.
        swerveDrive.setHeadingCorrection(false);

        // Configure PathPlanner.
        RobotConfig robotConfig = null;
        try {
            // Load the config from the GUI's settings menu.
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            throw new RuntimeException("Could not load path planner config!", e);
        }

        // Configure the AutoBuilder, what PathPlanner uses to move the robot
        // automatically and follow paths. This method is scary looking, but I promise
        // it's not that complicated.
        AutoBuilder.configure(
                // A function to get the position of the robot,
                swerveDrive::getPose,
                // A function to reset the position the robot thinks it is at,
                swerveDrive::resetOdometry,
                // A function to get the velocity of the robot,
                swerveDrive::getRobotVelocity,
                // A function to drive the robot given some target speed (feedforwards are
                // unused),
                (speeds, feedforwards) -> swerveDrive.drive(speeds),
                // The swerve's PID config to maintain those speeds,
                Constants.Drive.SWERVE_CONFIG,
                // The PathPlanner GUI setting config,
                robotConfig,
                // A function to determine if the path should be flipped because we are on the
                // red alliance,
                PathPlannerHelpers::shouldFlip,
                // And a reference to ourself.
                this);
        
        // Make sure nothing went wrong.
        assert (AutoBuilder.isConfigured());
        assert (swerveDrive != null);
    }

    @Override
    public void periodic() {
        // Update the swerveDrive's guess of our position based on the gyroscope.
        swerveDrive.updateOdometry();

        // Figure out how much left the stick is moved.
        double leftMag = Math.hypot(RobotContainer.primaryController.getLeftX(),
                RobotContainer.primaryController.getLeftY());

        // Figure out how much x axis of the right the stick is moved.
        double rightMag = Math.abs(RobotContainer.primaryController.getRightX());

        // if the driver is pushing on the control stick, then cancel the pathfind.
        if (leftMag > Constants.Drive.CONTROL_OVERRIDE_THRESHOLD || rightMag > Constants.Drive.CONTROL_OVERRIDE_THRESHOLD) {
            PathPlannerHelpers.cancelPathfind();
        }
    }

    /**
     * Vrrrrooooooooom brrrrrrrrr BRRRRRR wheeee BRRR brrrr VRRRRROOOOOOM ZOOOOOOM
     * ZOOOOM WAHOOOOOOOOO
     * WAHAHAHHA (Drives given a desired translation and rotation.)
     *
     * @param translation   Desired translation in meters per second.
     * @param rotation      Desired rotation in radians per second.
     * @param fieldRelative If not field relative, the robot will move relative to
     *                      its own rotation.
     */
    public static void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(
                // Invert the translation command if needed.
                translation.times(driveInverted ? -1 : 1),
                // Invert the rotation command if needed.
                rotation * (rotationInverted ? -1 : 1),
                fieldRelative,
                false);
    }

    

    // --- Utils ---

    /**
     * Reset the Robot's odometry rotation if it gets a significant drift.
     */
    public static void resetRotation() {
        // Set the gyro to 0,
        gyroscope.reset();
        // Then reset the swerve odometry.
        resetOdometry(swerveDrive.getPose());
    }

    /**
     * Sets the robot's odometry to a given pose with rotation at 0.
     * 
     * @param pose2d The pose to reset the odometry to.
     */
    public static void resetOdometry(Pose2d pose2d) {
        swerveDrive.resetOdometry(new Pose2d(pose2d.getTranslation(), new Rotation2d(0)));
    }

    /**
     * @return The robot's current rotation as reported by odometry.
     */
    public static Angle getRotation() {
        return Angle.ofBaseUnits(getPose().getRotation().getRadians(), Radians);
    }

    /**
     * @return The robot's current position as reported by odometry.
     */
    public static Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Inverts the rotation controls.
     */
    public static void invertRotation() {
        rotationInverted = !rotationInverted;
    }

    /**
     * Inverts the drive controls.
     */
    public static void invertDrive() {
        driveInverted = !driveInverted;
    }

    /**
     * Fetch the SimpleSwerveCommands instance
     */
    public static SwerveDriveCommands getCommands() {
        return commands;
    }

    /**
     * Fetch the SwerveDrive instance
     */
    public static SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }
}
