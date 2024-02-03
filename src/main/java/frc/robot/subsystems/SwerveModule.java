// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


// import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.CANcoderConfigurator;
// import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  // Create our turn and drive motors
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  // Creates the PID Controller for turning the motors
  private final PIDController turningPidController;

  // absolute encoders track the position of the motor regardless of power cycles
  private final CANcoder canCoder;
  
  private final double absoluteEncoderOffset;
  private final boolean absoluteEncoderReversed;
  // private final double absoluteEncoderOffsetRad;

  // Public constructor that takes in ids, offsets, and whether the motors are reversed. 
  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    // Offset of the absolute encoder should be value that is passed in. 
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    this.absoluteEncoderOffset = absoluteEncoderOffset;
    canCoder = new CANcoder(absoluteEncoderId);


    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

    driveMotor.setInverted(driveMotorReversed);
    driveMotor.setSmartCurrentLimit(30);
    turningMotor.setInverted(turningMotorReversed);
    turningMotor.setSmartCurrentLimit(30);


    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);

    // Min and max points are considered to be the same point; encoder values are continous around a full circle
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    // Reset encoders to be where the absolute encoder is minus offset. 
    resetEncoders();
  }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getTurningPosition() {
    return turningEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getTurningVelocity() {
    return turningEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRot() {
    double angle = canCoder.getAbsolutePosition().getValueAsDouble(); 
    angle += absoluteEncoderOffset;
    // angle = Units.degreesToRotations(angle);
    //  angle -= absoluteEncoderOffsetRad;
    // Set angle to be multiplied by -1 if the it's reversed
    // System.out.println(angle);
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(Units.rotationsToRadians(getAbsoluteEncoderRot())); // Takes in rotation found by absolute encoder. 
  }

  // Return module's drive and turning positions. 
  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(getTurningPosition()));
    }

  // Return a vector-like state that has velocity and angle as magnitude and angle. 
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition())); 
  }

  public double getDriveCurrent(){
    return driveMotor.getOutputCurrent();
  }

  public double getTurnCurrent(){
    return turningMotor.getOutputCurrent();
  }


  // Method that translate desired state to angle rotations and speed. 
  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    // Ensure that max rotation for turning motor is 90 degrees. 
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    // Clamp our speed to be between -1 and 1. 
    turningMotor.set(MathUtil.clamp(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()), -1, 1));
    SmartDashboard.putNumber("Swerve[" + turningMotor.getDeviceId() + "] Drive Speed", state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    SmartDashboard.putNumber("Swerve[" + turningMotor.getDeviceId() + "] Turn Output", turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));    
    SmartDashboard.putNumber("Swerve[" + turningMotor.getDeviceId() + "] Turn Target", state.angle.getDegrees());    
    
    SmartDashboard.putString("Swerve[" + driveMotor.getDeviceId() + "] state", state.toString());
  }
  
  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  @Override
  public void periodic() {
  }
}
