/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DistanceSensorSubsystem;

public class SensorSlowCommand extends CommandBase {
  private final DistanceSensorSubsystem m_distanceSensorSubsystem;
  private final DriveSubsystem m_driveSubsystem;

  private double slowThreshold = 30.0;
  private double stopThreshold = 4.0;
  private Timer timeSinceLastReading;
  private double currReading;
  /**
   * Creates a new SensorSlowCommand.
   */
  public SensorSlowCommand(DistanceSensorSubsystem distanceSensorSubsystem, DriveSubsystem driveSubsystem, XboxController xboxController) {
    m_driveSubsystem = driveSubsystem;
    m_distanceSensorSubsystem = distanceSensorSubsystem;
    timeSinceLastReading = new Timer();
    addRequirements(m_distanceSensorSubsystem, m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeSinceLastReading.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("Timer", timeSinceLastReading.get());

    if(timeSinceLastReading.get() >= 0.02) {
      currReading = m_distanceSensorSubsystem.getFrontRightDistance();
      timeSinceLastReading.reset();
    } 
   
    if(currReading < slowThreshold) {
      m_driveSubsystem.setSpeedMultiplier(0.5);
    } else {
      m_driveSubsystem.setSpeedMultiplier(1.0);
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.setSpeedMultiplier(1.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currReading <= stopThreshold;
  }
}
