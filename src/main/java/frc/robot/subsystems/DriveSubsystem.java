/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
   
    private WPI_TalonSRX l1 = new WPI_TalonSRX(Constants.DRIVESUBSYSTEM_LEFT_BACK_TALON), r1 = new WPI_TalonSRX(Constants.DRIVESUBSYSTEM_RIGHT_BACK_TALON);
    private WPI_VictorSPX l2= new WPI_VictorSPX(Constants.DRIVESUBSYSTEM_LEFT_FRONT_VICTOR), r2 = new WPI_VictorSPX(Constants.DRIVESUBSYSTEM_RIGHT_FRONT_VICTOR);
    private SpeedControllerGroup left = new SpeedControllerGroup(l1,l2), right = new SpeedControllerGroup(r1,r2);
    private DifferentialDrive drive = new DifferentialDrive(left, right);
    private double speedMultiplier;
    private boolean toggle = true;
    private XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);
    private final Logger logger = Logger.getLogger(this.getClass().getName());
    
  /**
   * Creates a new DriveSubsystem.
   * 
   * Speed Multiplier starts at 1.0
   */
  public DriveSubsystem() {
    resetSpeedMultiplier();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler 
  }

  public void setSpeedMultiplier(Double speed) {
    speedMultiplier = speed;
  }

  public double getSpeedMultiplier() {
    return speedMultiplier;
  }

  public void resetSpeedMultiplier() { 
    // Defaults speedMultiplier to 1.0
    speedMultiplier = 1.0;
  }

  public void teleOpDrive(double straightSpeed, double turnSpeed) {
    // Drives at a forward speed and rotational speed
    drive.arcadeDrive(straightSpeed * speedMultiplier, (turnSpeed * 0.85) * speedMultiplier);
    logger.warning("Speed: " + straightSpeed + "SpeedMultiplier: " + speedMultiplier);
    // drive.arcadeDrive(straightSpeed * speedMultiplier, turnSpeed * Constants.SLOW_TURN_MULTIPLE * speedMultiplier);
    // The slow turn multiple makes the turning too slow at half speed, so we have commented it out for now.
  }

  public void turnDriveAtSpeed(double speed) {
    l1.set(ControlMode.PercentOutput, speed);
    l2.set(ControlMode.PercentOutput, speed);
    r1.set(ControlMode.PercentOutput, speed);
    r2.set(ControlMode.PercentOutput, speed);
  }

  public void straightDrive(double speed) {
    l1.set(ControlMode.PercentOutput, -speed);
    l2.set(ControlMode.PercentOutput, -speed);
    r1.set(ControlMode.PercentOutput, speed);
    r2.set(ControlMode.PercentOutput, speed);
  }
}
