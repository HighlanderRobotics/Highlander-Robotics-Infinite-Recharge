/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DistanceSensorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

public class AutoAim extends CommandBase {

  private final DriveSubsystem m_driveSubsystem;
  private final LimeLightSubsystem m_limeLightSubsystem;
  private final DistanceSensorSubsystem m_distanceSensorSubsystem;
  private final PIDController turnPID = new PIDController(0.1, 0, 0);
  /**
   * Creates a new autoAim.
   */
  public AutoAim(DriveSubsystem driveSubsystem, LimeLightSubsystem limelightSubsystem, DistanceSensorSubsystem distanceSensorSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_limeLightSubsystem = limelightSubsystem;
    m_distanceSensorSubsystem = distanceSensorSubsystem;
    addRequirements(m_driveSubsystem, m_limeLightSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Create PIDController as an instance variable
    // Set kI and kD to 0 intiially
    // Set kP to some small value to start
    //
    //  -30 off
    //  * 0.01 = -0.3
    //  30
    //  * 0.01 = 0.3
    //  
    //  err_value * kP + integral(err_value) * kI + derivative(err_value) * kD

    

    if(m_limeLightSubsystem.getHorizontalOffset() > 7) {
      m_driveSubsystem.turnDriveAtSpeed(0.3);
    } else if(m_limeLightSubsystem.getHorizontalOffset() < -7) {
      m_driveSubsystem.turnDriveAtSpeed(-0.3);
    } else {
      if(m_limeLightSubsystem.getArea() < 25)
        m_driveSubsystem.straightDrive(-0.3);
      else if(m_distanceSensorSubsystem.getFrontRightDistance() >= 5)
        m_driveSubsystem.straightDrive(0.3);
      else
        m_driveSubsystem.straightDrive(0);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_distanceSensorSubsystem.getFrontRightDistance() <= 5;
  }
}