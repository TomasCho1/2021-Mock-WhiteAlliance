// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import fr.robot.subsystems.White_DriveTrain;
import frc.robot.Constants;

public class ArcadeDrive extends CommandBase {
  private final White_DriveTrain white_DriveTrain;
  private final Joystick joystick; 

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(White_DriveTrain dt, Joystick j) {
    white_DriveTrain = dt;
    joystick = j; 
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(white_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    white_DriveTrain.arcadeDrive(0.8 * joystick.getRawAxis(Constants.JoystickAxis.XAxis),
    -0.8 joystick.getRawAxis(Constants.JoystickAxis.YAxis)); 
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
