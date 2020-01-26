/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsytem extends SubsystemBase {
  /**
   * Creates a new IntakeSubsytem.
   */
  private final VictorSPX intakeMotor = new VictorSPX(7);

  public IntakeSubsytem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
