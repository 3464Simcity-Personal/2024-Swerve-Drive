// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class RumbleCMD extends Command {
  /** Creates a new RumbleCMD. */
  private final XboxController xbox = Constants.OperatorConstants.xbox;
  private final SwerveSubsystem swerveSub;

  public RumbleCMD(SwerveSubsystem swerveSub) {
    this.swerveSub = swerveSub;
    addRequirements(swerveSub);
  
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSub.rumbleDude(Math.abs(xbox.getRawAxis(3)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
