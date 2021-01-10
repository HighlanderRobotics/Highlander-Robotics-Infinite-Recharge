/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimeLightSubsystem extends SubsystemBase {

  public double horizontalOffset; 
  public double verticalOffset;
  public double areaOffset;
  NetworkTable table;
  
  public LimeLightSubsystem() {

  }

  public void defaultReadings() {
    

    //read values periodically
    double x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    double y = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
    double area = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);


    
  }

  public void lightReadings() {
    
  }

  public double getArea() {
    return areaOffset;
  }

  public double getHorizontalOffset() {
    return horizontalOffset;
  }

  public double getVerticalOffset() {
    return verticalOffset;
  }

  public void lightOn() {
    NetworkTableInstance.getDefault().getTable("limelight-high").getEntry("ledMode").setNumber(0);
  }

  public void lightOff() {
    NetworkTableInstance.getDefault().getTable("limelight-high").getEntry("ledMode").setNumber(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double x = NetworkTableInstance.getDefault().getTable("limelight-high").getEntry("tx").getDouble(0.0);
    double y = NetworkTableInstance.getDefault().getTable("limelight-high").getEntry("ty").getDouble(0.0);
    double area = NetworkTableInstance.getDefault().getTable("limelight-high").getEntry("ta").getDouble(0.0);
    SmartDashboard.putNumber("limelightX", x);
    SmartDashboard.putNumber("limelightY", y);
    SmartDashboard.putNumber("limelightArea", area);

    horizontalOffset = x;
    verticalOffset = y;

  }

}
