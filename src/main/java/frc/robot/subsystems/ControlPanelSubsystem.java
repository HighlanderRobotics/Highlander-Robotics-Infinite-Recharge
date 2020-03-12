/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import frc.robot.Constants;

public class ControlPanelSubsystem extends SubsystemBase {
  
  // Color Sensor setup
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.361, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.321, 0.333, 0.154);
  private final Color kYellowTarget = ColorMatch.makeColor(0.325, 0.594, 0.113);

  // ESC's
  private VictorSPX panelMotor = new VictorSPX(Constants.CONTROLPANELSUBSYSTEM_VICTOR);

  //Pistons


   /**
   * Creates a new ExampleSubsystem.
   */
  public ControlPanelSubsystem() {
    
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
  }


  public void zeroSpeed() {
    panelMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void setSpeed(double speed) {
    panelMotor.set(ControlMode.PercentOutput, speed);
  }

  public void colorRotation(String c) {
    while(!isMatchingColor(c)) {
      panelMotor.set(ControlMode.PercentOutput, 0.5);
    }
  }

  public void quarterSpeed() {
    panelMotor.set(ControlMode.PercentOutput, 0.25);
  }

  public void halfSpeed() {
    panelMotor.set(ControlMode.PercentOutput, 0.5);
  }

  public void threeQuarterSpeed() {
    panelMotor.set(ControlMode.PercentOutput, 0.75);
  }

  public void fullSpeed() {
    panelMotor.set(ControlMode.PercentOutput, 1.0);
  }
  
  public boolean isMatchingColor(String c) {
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    if(match.color == kRedTarget)
      return c.equals("R");
    else if(match.color == kGreenTarget)
      return c.equals("G");
    else if(match.color == kBlueTarget)
      return c.equals("B");
    else if(match.color == kYellowTarget)
      return c.equals("Y");
    else
      return false;
  }

  public String colorDetectedPosistion(){
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    if(match.color == kRedTarget)
      return "B";
    else if(match.color == kGreenTarget)
      return "Y";
    else if(match.color == kBlueTarget)
      return "R";
    else if(match.color == kYellowTarget)
      return "G";
    return "none";
  }

  //Rotation Control Psuedo Code
  /*
  *Create method that detects a color and returns a string corresponding to it
  */
  public String colorDetected(){
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    if(match.color == kRedTarget)
      return "R";
    else if(match.color == kGreenTarget)
      return "G";
    else if(match.color == kBlueTarget)
      return "B";
    else if(match.color == kYellowTarget)
      return "Y";
    return "none";
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Color", colorDetected());

    // System.out.println(m_colorSensor.getRed() + ", " + m_colorSensor.getGreen() + ", " + m_colorSensor.getBlue());
    // This method will be called once per scheduler run
  }
}
