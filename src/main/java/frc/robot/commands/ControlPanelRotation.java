/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

public class ControlPanelRotation extends CommandBase {
  private ControlPanelSubsystem m_controlPanelSubsystem = new ControlPanelSubsystem();
  /**
   * Creates a new ControlPanelPosistion.
   */
  //Creates a color string that can be used by both initialize() and execute()
  String startingColor = new String("");
  double prevTime;
  int counter;
  public ControlPanelRotation(ControlPanelSubsystem controlPanelSubsystem) {
    m_controlPanelSubsystem = controlPanelSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_controlPanelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_controlPanelSubsystem.isMatchingColor("R")){
      startingColor = "R";
    }
    else if(m_controlPanelSubsystem.isMatchingColor("G")){
      startingColor = "G";
    }
    else if(m_controlPanelSubsystem.isMatchingColor("B")){
      startingColor = "B";
    }
    else if(m_controlPanelSubsystem.isMatchingColor("Y")){
      startingColor = "Y";
    }
    prevTime = Timer.getFPGATimestamp();
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(counter != 6)
      m_controlPanelSubsystem.quarterSpeed();
    if(Timer.getFPGATimestamp() - prevTime >= 0.3){
      if(startingColor.equals(m_controlPanelSubsystem.colorDetected()))
        counter++;
        System.out.println(counter);
      if(counter == 6){
        m_controlPanelSubsystem.zeroSpeed();
      }
      prevTime = Timer.getFPGATimestamp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(counter == 6)
      return true;
    else 
      return false;
  }
}
