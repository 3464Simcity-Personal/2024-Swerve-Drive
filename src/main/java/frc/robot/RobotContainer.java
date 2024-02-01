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
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class RobotContainer {
  private final SendableChooser<String> commandChooser = new SendableChooser<>();
  public SequentialCommandGroup selectedAuto;
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
    // where we set the options that user has to choose for autos 
    commandChooser.setDefaultOption("Red 3 Amp ", "R3A");
    commandChooser.addOption("Red 3 Amp Hail Mary", "R3AHM");
    commandChooser.addOption("Red 3 Speaker", "R3S");
    SmartDashboard.putData("Auto", commandChooser);
    CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem, swerveCMD);
    configureBindings();
  }

 
  private void configureBindings() {
    Constants.OperatorConstants.buttonX.onTrue(resetGyro);
    Constants.OperatorConstants.buttonY.onTrue(new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)))));
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

    Trajectory tragOriginToAmp = TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(-90)), 
      new Pose2d(0.427, 0.451, Rotation2d.fromDegrees(-90))), trajectoryConfig); // Apply trajectory settings to path


    Trajectory tragAmpToAmpNote = TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(-90)), 
      new Pose2d(0.9, -0.6, Rotation2d.fromDegrees(-40)),
      new Pose2d(1.03, -0.74, Rotation2d.fromDegrees(-40))), trajectoryConfig);
      
    Trajectory tragAmpNoteToAmp = TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(-40)), 
      new Pose2d(-0.9, 0.6, Rotation2d.fromDegrees(90)),
      new Pose2d(-1.03, 0.74, Rotation2d.fromDegrees(-90))), trajectoryConfig);

    Trajectory tragAmpToSpeakerNote = TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(-90)), 
      new Pose2d(0.5, -2.178, Rotation2d.fromDegrees(-90)),
      new Pose2d(1.055, -2.178, Rotation2d.fromDegrees(0))), trajectoryConfig);

    Trajectory tragSpeakerNoteToAmp = TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      new Pose2d(-0.8, 1.2, Rotation2d.fromDegrees(-90)),
      new Pose2d(-1.055, 2.2, Rotation2d.fromDegrees(-90))), trajectoryConfig);

    Trajectory tragAmpToHailMaryNote = TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(-90)),
      new Pose2d(5.5, -0.282, Rotation2d.fromDegrees(0)),
      new Pose2d(6.424, -0.282, Rotation2d.fromDegrees(0))), trajectoryConfig);
    
    Trajectory tragHailMaryNoteToAmpM1 = TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
      new Pose2d(-6.4, -0.5, Rotation2d.fromDegrees(180)),
      new Pose2d(-6.4, 0.283, Rotation2d.fromDegrees(270))), trajectoryConfig);


      // contruct command to follow trajectory
      SwerveControllerCommand orginToAmp = new SwerveControllerCommand(
        tragOriginToAmp, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

      SwerveControllerCommand ampToSpeaker = new SwerveControllerCommand(
        tragAmpToSpeakerNote, 
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
      
      SwerveControllerCommand speakerNoteToAmp = new SwerveControllerCommand(
        tragSpeakerNoteToAmp, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

      SwerveControllerCommand ampToRightmostNote = new SwerveControllerCommand(
        tragAmpToHailMaryNote, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

      SwerveControllerCommand rightmostNoteToAmpM1 = new SwerveControllerCommand(
        tragHailMaryNoteToAmpM1, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);



    // ------------------- Points relative to speaker ----------------------- //

    Trajectory tragOriginToStageNote = TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
      new Pose2d(0.508, -1.4478, Rotation2d.fromDegrees(0)),
      new Pose2d(1.3462, -1.4478, Rotation2d.fromDegrees(0))), trajectoryConfig); // change X to 1.3464 because team spirit and nationalism 
   
      Trajectory tragStageNoteToSpeakerShooting = TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
      new Pose2d(-0.508, 1.2954, Rotation2d.fromDegrees(0)),
      new Pose2d(-0.508, 1.2954, Rotation2d.fromDegrees(0))), trajectoryConfig); // change X to 1.3464 because team spirit and nationalism 

      Trajectory tragSpeakerShootingToSpeakerNote = TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
      new Pose2d(0.6, 0, Rotation2d.fromDegrees(0))), trajectoryConfig); // change X to 1.3464 because team spirit and nationalism 

    
    SwerveControllerCommand originToStageNote = new SwerveControllerCommand(
        tragOriginToStageNote, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);
        
    SwerveControllerCommand stageNoteToSpeakerShooting = new SwerveControllerCommand(
        tragStageNoteToSpeakerShooting, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);

      SwerveControllerCommand speakerShootingToSpeakerNote = new SwerveControllerCommand(
        tragSpeakerShootingToSpeakerNote, 
        swerveSubsystem::getPose, // Coords
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates, // Function to translate speeds to the modules
        swerveSubsystem);
    

    // Start HERE:
    if (commandChooser.getSelected() == "R3A"){
      // add some init and wrap up, and return everything
      selectedAuto = new SequentialCommandGroup(
        // Reset odometry to starting pose. 
        new InstantCommand(() -> swerveSubsystem.resetOdometry(tragOriginToAmp.getInitialPose())),
        orginToAmp,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(tragAmpToAmpNote.getInitialPose())),
        ampToAmpN,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(tragAmpNoteToAmp.getInitialPose())),
        ampNToAmp,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(tragAmpToSpeakerNote.getInitialPose())),
        ampToSpeaker,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(tragSpeakerNoteToAmp.getInitialPose())),
        speakerNoteToAmp,
        new InstantCommand(() -> swerveSubsystem.stopModules()));

    }else if (commandChooser.getSelected() == "R3AHM"){
        // HAIL MARY
            // add some init and wrap up, and return everything
        selectedAuto = new SequentialCommandGroup(
          // Reset odometry to starting pose. 
          new InstantCommand(() -> swerveSubsystem.resetOdometry(tragOriginToAmp.getInitialPose())),
          orginToAmp,
          new InstantCommand(() -> swerveSubsystem.stopModules()),
          new WaitCommand(0.25),
          new InstantCommand(() -> swerveSubsystem.resetOdometry(tragAmpToAmpNote.getInitialPose())),
          ampToAmpN,
          new InstantCommand(() -> swerveSubsystem.stopModules()),
          new WaitCommand(0.25),
          new InstantCommand(() -> swerveSubsystem.resetOdometry(tragAmpNoteToAmp.getInitialPose())),
          ampNToAmp,
          new InstantCommand(() -> swerveSubsystem.stopModules()),
          new WaitCommand(0.25),
          new InstantCommand(() -> swerveSubsystem.resetOdometry(tragAmpToHailMaryNote.getInitialPose())),
          ampToRightmostNote,
          new InstantCommand(() -> swerveSubsystem.stopModules()),
          new WaitCommand(0.25),
          new InstantCommand(() -> swerveSubsystem.resetOdometry(tragHailMaryNoteToAmpM1.getInitialPose())),
          new WaitCommand(0.25),
          rightmostNoteToAmpM1,
          new InstantCommand(() -> swerveSubsystem.stopModules())
        );
    }else if(commandChooser.getSelected() == "R3S"){
      selectedAuto = new SequentialCommandGroup(
        new InstantCommand(() -> swerveSubsystem.resetOdometry(tragOriginToStageNote.getInitialPose())),
        originToStageNote,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(tragStageNoteToSpeakerShooting.getInitialPose())),
        stageNoteToSpeakerShooting,
        new WaitCommand(0.25),
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(tragSpeakerShootingToSpeakerNote.getInitialPose())),
        speakerShootingToSpeakerNote,
        new InstantCommand(() -> swerveSubsystem.stopModules())
        );

    }else{
        selectedAuto = null;
    }


    return selectedAuto;
  }
}
