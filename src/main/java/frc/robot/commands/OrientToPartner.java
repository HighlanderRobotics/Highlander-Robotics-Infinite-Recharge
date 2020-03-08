/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.DriveSubsystem;

public class OrientToPartner extends CommandBase {
  private Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
  private final DriveSubsystem m_driveSubsystem;
  private double heading = gyro.getAngle();
  private int prospectiveAngle = -90; // -90 if left and 90 if right (I think)

  /**
   * Creates a new OrientToPartner.
   */
  public OrientToPartner(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_driveSubsystem.turnToAngle(prospectiveAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.turnDriveAtSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return prospectiveAngle - gyro.getAngle() <= Math.abs(0.5);
  }
}
