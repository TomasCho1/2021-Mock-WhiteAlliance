// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import fr.robot.subsystems.White_DriveTrain;
import frc.robot.RobotContainer;

public class PIDDriveForward extends CommandBase {
  double _Distance;
  /** Creates a new PIDDriveForward. */
  public PIDDriveForward(double Distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.getDriveTrain());
    _Distance = Distance;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.getDriveTrain().resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.getDriveTrain().tankDrive(0.5, 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.getDriveTrain().tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RobotContainer.getDriveTrain().GetDistance() >= _Distance){
      return true;
    }
    return false;
  }
}
