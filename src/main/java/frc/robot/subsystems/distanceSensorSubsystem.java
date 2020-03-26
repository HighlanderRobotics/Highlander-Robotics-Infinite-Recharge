package frc.robot.subsystems;
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/



import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DistanceSensorSubsystem extends SubsystemBase {
  /**
   * Creates a new distanceSensorSubsystem.
   */

  private final Ultrasonic front = new Ultrasonic(Constants.PING_CHANNEL, Constants.ECHO_CHANNEL);
  //private final Ultrasonic frontLeft = new Ultrasonic(2, 3);

  public DistanceSensorSubsystem() {
    front.setAutomaticMode(true);
    //frontLeft.setAutomaticMode(false);
  }

  public double getFrontRightDistance() {
    return front.getRangeInches();
  }

 /*
  public double getFrontLeftDistance() {
    frontLeft.ping();
    return frontLeft.getRangeInches();
  }
  */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
