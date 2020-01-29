/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

public class ControlPanelPosition extends CommandBase {
  private final ControlPanelSubsystem m_controlPanelSubsystem;
    List<String> colors = Arrays.asList("R", "G", "B", "Y");
    String color = colors.get((int)(Math.random()*4));
    //randomColor.nextInt(4)
  public ControlPanelPosition(ControlPanelSubsystem controlPanelSubsystem) {
    m_controlPanelSubsystem = controlPanelSubsystem;
    addRequirements(m_controlPanelSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_controlPanelSubsystem.isMatchingColor(color))
      m_controlPanelSubsystem.quarterSpeed();
    else{
      m_controlPanelSubsystem.zeroSpeed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_controlPanelSubsystem.isMatchingColor(color))
      return true;
    else
      return false;
  }
}
