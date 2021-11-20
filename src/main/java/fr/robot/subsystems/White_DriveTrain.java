// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package fr.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class White_DriveTrain extends SubsystemBase {

  private final WPI_TalonSRX _leftDriveTalon;
  private final WPI_TalonSRX _righttDriveTalon;
  

  private DifferentialDrive _diffDrive;
  private double WheelRadius = 0.0762;

  /** Creates a new DriveTrain. */
  public White_DriveTrain() {
    _leftDriveTalon = new WPI_TalonSRX(Constants.LeftDriveTalonPort);
    _righttDriveTalon = new WPI_TalonSRX(Constants.RightDriveTalonPort);

    _leftDriveTalon.setInverted(false);
    _righttDriveTalon.setInverted(false);
    _leftDriveTalon.configFactoryDefault();
    _righttDriveTalon.configFactoryDefault();
    _leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    _righttDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    _diffDrive = new DifferentialDrive(_leftDriveTalon, _righttDriveTalon);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Distance", GetDistance());
  }
  public void resetEncoders() {
    _leftDriveTalon.setSelectedSensorPosition(0, 0, 10);
    _righttDriveTalon.setSelectedSensorPosition(0, 0, 10);

  }
  public void tankDrive(double leftSpeed, double rightSpeed) {
    _diffDrive.tankDrive(leftSpeed, rightSpeed);
  }
  public void arcadeDrive(double xSpeed, double zRotation) {
    _diffDrive.arcadeDrive(xSpeed, zRotation);

  }
  public double GetDistance() {
    double AverageTicks = (_leftDriveTalon.getSelectedSensorPosition(0) * -1 + _righttDriveTalon.getSelectedSensorPosition(0)) * 0.5;
    double Rotation = AverageTicks / 4096; 
    double Distance = 2 * Math.PI * WheelRadius * Rotation; 
    return Distance; 
  }
}
