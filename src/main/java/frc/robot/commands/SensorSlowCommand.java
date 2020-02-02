/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DistanceSensorSubsystem;

public class SensorSlowCommand extends CommandBase {
  private final DistanceSensorSubsystem m_distanceSensorSubsystem;
  private final DriveSubsystem m_driveSubsystem;
  private final XboxController driveController;

  private double slowThreshold = 30.0;
  private double stopThreshold = 1.5;
  private Timer readingDelay = new Timer();
  private double currReading;
  /**
   * Creates a new SensorSlowCommand.
   */
  public SensorSlowCommand(DistanceSensorSubsystem distanceSensorSubsystem, DriveSubsystem driveSubsystem, XboxController xboxController) {
    m_driveSubsystem = driveSubsystem;
    m_distanceSensorSubsystem = distanceSensorSubsystem;
    driveController = xboxController;
    addRequirements(m_distanceSensorSubsystem, m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    readingDelay.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currReading = m_distanceSensorSubsystem.getFrontRightDistance();
    Timer.delay(0.2);
    if(currReading < stopThreshold) {
      m_driveSubsystem.straightDrive(0.0);
    } else {
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
    if(m_distanceSensorSubsystem.getFrontRightDistance() < stopThreshold) {
      return true;
    } else {
      return false;
    }
  }
}
