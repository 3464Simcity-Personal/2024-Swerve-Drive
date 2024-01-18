// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCMD;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;


public class RobotContainer {
  private final XboxController xbox = new XboxController(3);

  
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final InstantCommand resetGyro = new InstantCommand(swerveSubsystem::zeroHeading, swerveSubsystem);
  private final SwerveJoystickCMD swerveCMD = new SwerveJoystickCMD(swerveSubsystem,
                () -> -xbox.getRawAxis(OIConstants.kDriverYAxis),
                () -> xbox.getRawAxis(OIConstants.kDriverXAxis),
                () -> xbox.getRawAxis(OIConstants.kDriverRotAxis),
                () -> true/*
                () -> !xbox.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx */);

  public RobotContainer() {
    CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem, swerveCMD);
    configureBindings();
  }

 
  private void configureBindings() {
    
    Constants.OperatorConstants.buttonX.onTrue(resetGyro);
    
  }

  public Command getAutonomousCommand() {
    // Trajectory Config for settings such as speed. 
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
          .setKinematics(DriveConstants.kDriveKinematics);

    // Trajectory Generation using WPILIB
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)), // Starting Pose
      List.of(
        new Translation2d(1, .5), new Translation2d(2, -.5)), 
        // new Pose2d(3, 0, Rotation2d.fromDegrees(180)),
        new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
        trajectoryConfig); // Apply trajectory settings to path

      // define pid controllers for tracking trajectory = creates speeds to correct for error. 
      PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
      PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);

      // Profiled PID Controller = PID Controller with constraints on max speed / acceleration. 
      ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController,
        0,
        0,
        AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);  

      // contruct command to follow trajectory
      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

    // add some init and wrap up, and return everything
    return new SequentialCommandGroup(
      // Reset odometry to starting pose. 
      new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> swerveSubsystem.stopModules())
    );
  }
}
