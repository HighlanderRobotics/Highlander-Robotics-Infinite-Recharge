/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DistanceSensorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import io.github.oblarg.oblog.annotations.Log;

public class ColorWheelApproach extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final DistanceSensorSubsystem m_distanceSensorSubsystem;
  @Log private boolean isPanelMovementFinished;
  
  public ColorWheelApproach(DriveSubsystem driveSubsystem, DistanceSensorSubsystem distanceSensorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    m_distanceSensorSubsystem = distanceSensorSubsystem;
    addRequirements(m_driveSubsystem, m_distanceSensorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isPanelMovementFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_distanceSensorSubsystem.getControlPanelDistance() > 20) {
      m_driveSubsystem.straightDrive(0.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isPanelMovementFinished = !interrupted;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_distanceSensorSubsystem.getControlPanelDistance() < 5;
  }
}
