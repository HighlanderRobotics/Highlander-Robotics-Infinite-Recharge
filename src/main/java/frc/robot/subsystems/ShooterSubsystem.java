/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsytem.
   */
  private VictorSPX backShooter = new VictorSPX(Constants.SHOOTERSUBSYSTEM_BACK_VICTOR);
  private VictorSPX frontShooter = new VictorSPX(Constants.SHOOTERSUBSYSTEM_FRONT_VICTOR);
  
  public ShooterSubsystem() {
    
  }

  public void shootAtSpeed(Double loader, Double shooter) {
    backShooter.set(ControlMode.PercentOutput, loader);
    frontShooter.set(ControlMode.PercentOutput, shooter);
  }

  public void shootBalls() {
    backShooter.set(ControlMode.PercentOutput, -0.5);
    frontShooter.set(ControlMode.PercentOutput, -0.7); //-0.436
  }

  public void zeroSpeed() {
    backShooter.set(ControlMode.PercentOutput, 0.0);
    frontShooter.set(ControlMode.PercentOutput, 0.0);
  }

  public void frontGiveBallSpeed() {
    frontShooter.set(ControlMode.PercentOutput, 0.1);
  }

  public void backQuarterSpeed() {
    backShooter.set(ControlMode.PercentOutput, -0.25);
  }

  public void frontQuarterSpeed() {
    frontShooter.set(ControlMode.PercentOutput, 0.25);
  }

  public void backHalfSpeed() {
    backShooter.set(ControlMode.PercentOutput, -0.5);
  }

  public void frontHalfSpeed() {
    frontShooter.set(ControlMode.PercentOutput, 0.5);
  }

  public void backThreeQuarterSpeed() {
    backShooter.set(ControlMode.PercentOutput, -0.75);
  }

  public void frontThreeQuarterSpeed() {
    frontShooter.set(ControlMode.PercentOutput, 0.75);
  }

  public void backFullSpeed() {
    backShooter.set(ControlMode.PercentOutput, -1.0);
  }

  public void frontFullSpeed() {
    frontShooter.set(ControlMode.PercentOutput, -1.0);
  }

  public void shoot() {
    frontShooter.set(ControlMode.PercentOutput, 1.0);
    backShooter.set(ControlMode.PercentOutput, 1.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}