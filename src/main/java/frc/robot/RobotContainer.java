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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class RobotContainer {
  private final XboxController xbox = new XboxController(3);

  
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final InstantCommand resetGyro = new InstantCommand(swerveSubsystem::zeroHeading, swerveSubsystem);
  // private final InstantCommand resetLocation = new InstantCommand(swerveSubsystem::resetOdom)
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
    Constants.OperatorConstants.buttonY.onTrue(new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-90)))));
  }

  public Command getAutonomousCommand() {
    // Trajectory Config for settings such as speed. 
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
          .setKinematics(DriveConstants.kDriveKinematics);

   
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


    // Start HERE:

    Trajectory tragOriginToAmptoNote = TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(-90)), 
      new Pose2d(0.427, 0.451, Rotation2d.fromDegrees(-90))),
      trajectoryConfig); // Apply trajectory settings to path

      Trajectory tragAmpToAmpNote = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(-90)), 
      new Pose2d(0.9, -0.6, Rotation2d.fromDegrees(-40)),
      new Pose2d(1.03, -0.74, Rotation2d.fromDegrees(-40))), trajectoryConfig);
    // Trajectory tragAmpToAmpNote = TrajectoryGenerator.generateTrajectory(
    //   new Pose2d(0, 0, Rotation2d.fromDegrees(-90)), 
    //   List.of(new Translation2d(0.5, -0.1), new Translation2d(1.03,-0.15)), 
    //   new Pose2d(1.03, -0.74, Rotation2d.fromDegrees(-90)), trajectoryConfig);
    //  // Trajectory Generation using WPILIB
      
    Trajectory tragAmpNoteToAmp = TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(-90)), 
      new Pose2d(-1.03, 0.74, Rotation2d.fromDegrees(-90))), trajectoryConfig);
    //  Trajectory tragAmpNoteToAmp = TrajectoryGenerator.generateTrajectory(
    //   new Pose2d(0, 0, Rotation2d.fromDegrees(-90)), 
    //   List.of(new Translation2d(-0.5, 0.37), new Translation2d(-1.03,0.38)), 
    //   new Pose2d(-1.03, 0.74, Rotation2d.fromDegrees(-90)), trajectoryConfig);
    
    // Trajectory testWooHoo = TrajectoryGenerator.generateTrajectory(
    //   List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
    //   new Pose2d(1, 0, Rotation2d.fromDegrees(0))), 
    //   trajectoryConfig);

          // contruct command to follow trajectory
      SwerveControllerCommand orginToAmp = new SwerveControllerCommand(
        tragOriginToAmptoNote, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

      SwerveControllerCommand ampToAmpN = new SwerveControllerCommand(
        tragAmpToAmpNote, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

      SwerveControllerCommand ampNToAmp = new SwerveControllerCommand(
        tragAmpNoteToAmp, 
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
      new InstantCommand(() -> swerveSubsystem.resetOdometry(tragOriginToAmptoNote.getInitialPose())),
      orginToAmp,
      new InstantCommand(() -> swerveSubsystem.stopModules()),
      new WaitCommand(0.25),
      new InstantCommand(() -> swerveSubsystem.resetOdometry(tragAmpToAmpNote.getInitialPose())),
      ampToAmpN,
      new WaitCommand(0.25),
      new InstantCommand(() -> swerveSubsystem.resetOdometry(tragAmpNoteToAmp.getInitialPose())),
      ampNToAmp,
      new InstantCommand(() -> swerveSubsystem.stopModules())
    );
  }
}
