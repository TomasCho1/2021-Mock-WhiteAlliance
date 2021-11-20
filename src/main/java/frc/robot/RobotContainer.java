// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import fr.robot.subsystems.White_DriveTrain;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import java.util.Arrays;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Joystick j1 = new Joystick(Constants.JoystickUSB.LeftJoy);
  private Joystick j2 = new Joystick(Constants.JoystickUSB.RightJoy);
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private static final White_DriveTrain DriveTrain = new White_DriveTrain();

  private final TankDrive tankDrive = new TankDrive(DriveTrain, j1, j2 );
  private final SequentialCommandGroup YellowPath;
  private final PIDturn PIDTurn; 
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriveTrain.setDefaultCommand(tankDrive);

    PIDTurn = new PIDturn(DriveTrain, 90);

    YellowPath = new SequentialCommandGroup(new PIDDriveForward(0.5), PIDTurn, new PIDDriveForward(-0.5));

    // Configure the button bindings

    
    configureButtonBindings();
  }
  public static White_DriveTrain getDriveTrain(){
    return DriveTrain;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new PIDDriveForward(1);
    
  }
}
