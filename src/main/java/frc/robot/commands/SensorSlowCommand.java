/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.distanceSensorSubsystem;

public class SensorSlowCommand extends CommandBase {
  private final distanceSensorSubsystem m_distanceSensorSubsystem;
  private final DriveSubsystem m_driveSubsystem;

  private int threshold = 30;
  /**
   * Creates a new SensorSlowCommand.
   */
  public SensorSlowCommand(distanceSensorSubsystem DistanceSensorSubsystem, DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_distanceSensorSubsystem = DistanceSensorSubsystem;
    addRequirements(m_distanceSensorSubsystem, m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_distanceSensorSubsystem.getFrontRightDistance() < threshold || m_distanceSensorSubsystem.getFrontLeftDistance() < threshold) {
      m_driveSubsystem.straightDrive(0.25);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_distanceSensorSubsystem.getFrontRightDistance() < threshold || m_distanceSensorSubsystem.getFrontLeftDistance() < threshold) {
      return false;
    } else {
      return true;
    }

  }
}
