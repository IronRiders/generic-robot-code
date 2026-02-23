// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.core;

import com.pathplanner.lib.auto.AutoBuilder;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drive.swerve.SwerveDriveCommands;
import frc.robot.drive.swerve.SwerveDriveSubsystem;
import frc.robot.lib.Constants;
import frc.robot.lib.motor.Motor;
import frc.robot.lib.motor.Motor.FeedforwardType;
import frc.robot.lib.motor.Motor.SupportedMotorType;

public class RobotContainer {
    // Create a "SendableChooser", basically just a dropdown menu that can be set in
    // the Dashboard.
    private static SendableChooser<Command> autoChooser;

    // Create the controller object.
    public static CommandXboxController primaryController = new CommandXboxController(
            Constants.Identifiers.PRIMARY_CONTROLLER_PORT);

    // Create our subsystems.
    public static SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
    public static SwerveDriveCommands swerveDriveCommands = SwerveDriveSubsystem.getCommands();

    public RobotContainer() {
        // Set the options based on what PathPlanner autos we have configured.
        autoChooser = AutoBuilder.buildAutoChooser();
        // Put the dropdown to the Dashboard.
        SmartDashboard.putData("Auto Select", autoChooser);

        // Log DriverStation messages.
        DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
        // Log PDH (Power Distribution Hub) messages.
        DogLog.setPdh(new PowerDistribution());

        new Motor(1, SupportedMotorType.NEO);

        configureBindings();
    }

    private void configureBindings() {
        // Set the default command to the drive command
        swerveDriveSubsystem.setDefaultCommand(
                swerveDriveCommands
                        .driveTeleop(
                                () -> primaryController.getLeftY(),
                                () -> primaryController.getLeftX(),
                                () -> primaryController.getRightX(),
                                true)
                        .withName("Drive Teleop"));

        // -- Example pathfinding commands --
        primaryController.rightBumper().onTrue(swerveDriveCommands.pathfindToPose(Constants.Field.CENTER));

        primaryController.leftBumper()
                .onTrue(swerveDriveCommands.pathfindToPose(new Pose2d(15, 4, new Rotation2d(Math.toRadians(255)))));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
