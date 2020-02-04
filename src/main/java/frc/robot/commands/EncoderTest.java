/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

public class EncoderTest extends CommandBase {

  private final ControlPanelSubsystem m_controlPanelSubsystem;
  private final Encoder encoder = new Encoder(0, 1);

  public EncoderTest(ControlPanelSubsystem controlPanelSubsystem) {
      m_controlPanelSubsystem = controlPanelSubsystem;
      addRequirements(m_controlPanelSubsystem);
      // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    encoder.reset();
    encoder.setDistancePerPulse(1./1024.);
    m_controlPanelSubsystem.quarterSpeed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {        
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controlPanelSubsystem.zeroSpeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return encoder.getDistance() >= 1;
  }
}