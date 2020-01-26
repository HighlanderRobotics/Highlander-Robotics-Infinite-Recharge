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

public class ShooterSubsytem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsytem.
   */
  private VictorSPX backShooter = new VictorSPX(5);
  private VictorSPX frontShooter = new VictorSPX(6);
  
  public ShooterSubsytem() {

  }

  public void zeroSpeed() {
    backShooter.set(ControlMode.PercentOutput, 0.0);
    frontShooter.set(ControlMode.PercentOutput, 0.0);
  }

  public void backQuarterSpeed() {
    backShooter.set(ControlMode.PercentOutput, 0.25);
  }

  public void frontQuarterSpeed() {
    frontShooter.set(ControlMode.PercentOutput, 0.25);
  }

  public void backHalfSpeed() {
    backShooter.set(ControlMode.PercentOutput, 0.5);
  }

  public void frontHalfSpeed() {
    frontShooter.set(ControlMode.PercentOutput, 0.5);
  }

  public void backThreeQuarterSpeed() {
    backShooter.set(ControlMode.PercentOutput, 0.75);
  }

  public void frontThreeQuarterSpeed() {
    frontShooter.set(ControlMode.PercentOutput, 0.75);
  }

  public void backFullSpeed() {
    backShooter.set(ControlMode.PercentOutput, 1.0);
  }

  public void frontFullSpeed() {
    frontShooter.set(ControlMode.PercentOutput, 1.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
