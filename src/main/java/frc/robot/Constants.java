// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    // Level1 Standard Gear Ratios
    // https://www.andymark.com/products/mk4i-swerve-modules
    public static final double kDriveMotorGearRatio = 1 / 8.14;
    public static final double kTurningMotorGearRatio = 1 / 21.43; // 150/7 : 1 gear ratio
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;
  }

  public static final class DriveConstants {
    public static final double kTrackWidth = Units.inchesToMeters(20.5);
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(26.5);
  
    // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    public static final int kFrontLeftDriveMotorPort = 8;
    public static final int kBackLeftDriveMotorPort = 2;
    public static final int kFrontRightDriveMotorPort = 6;
    public static final int kBackRightDriveMotorPort = 4;

    public static final int kFrontLeftTurningMotorPort = 7;
    public static final int kBackLeftTurningMotorPort = 1;
    public static final int kFrontRightTurningMotorPort = 5;
    public static final int kBackRightTurningMotorPort = 3;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 9;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 11;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 10;
    public static final int kBackRightDriveAbsoluteEncoderPort = 12;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 1.535;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 6.26;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 1.638;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 2.471;

    // Robot Speed Constraints
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
            kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
}

public static final class AutoConstants {
  public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
  public static final double kMaxAngularSpeedRadiansPerSecond = //
          DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
  public static final double kPXController = 1.5;
  public static final double kPYController = 1.5;
  public static final double kPThetaController = 3.75 ;

  public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
          new TrapezoidProfile.Constraints(
                  kMaxAngularSpeedRadiansPerSecond,
                  kMaxAngularAccelerationRadiansPerSecondSquared);
}

public static final class OIConstants {
  public static final int kDriverControllerPort = 0;

  public static final int kDriverYAxis = 1;
  public static final int kDriverXAxis = 0;
  public static final int kDriverRotAxis = 4;
  public static final int kDriverFieldOrientedButtonIdx = 1;

  public static final double kDeadband = 0.05;
}

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final Joystick auxStick = new Joystick(0);
    public static final JoystickButton button2 = new JoystickButton(auxStick, 2);
    public static final XboxController xbox = new XboxController(3);
    public static final JoystickButton buttonX = new JoystickButton(xbox, 3);
  }
}
