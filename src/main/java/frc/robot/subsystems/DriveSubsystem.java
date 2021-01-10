/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class DriveSubsystem extends SubsystemBase implements Loggable {
    private WPI_TalonSRX l1 = new WPI_TalonSRX(Constants.DRIVESUBSYSTEM_LEFT_BACK_TALON);  
    private WPI_VictorSPX l2 = new WPI_VictorSPX(Constants.DRIVESUBSYSTEM_LEFT_FRONT_VICTOR);
    private WPI_TalonSRX r1 = new WPI_TalonSRX(Constants.DRIVESUBSYSTEM_RIGHT_BACK_TALON);
    private WPI_VictorSPX r2 = new WPI_VictorSPX(Constants.DRIVESUBSYSTEM_RIGHT_FRONT_VICTOR);
    private SpeedControllerGroup left = new SpeedControllerGroup(l1,l2);
    private SpeedControllerGroup right = new SpeedControllerGroup(r1,r2);
    private DifferentialDrive drive = new DifferentialDrive(left, right);
    private double speedMultiplier;
    private final Logger logger = Logger.getLogger(this.getClass().getName());
    private Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
    private double kP = .0005;
    private double heading = gyro.getAngle();
    private SlewRateLimiter speedRateLimiter = new SlewRateLimiter(Constants.SLEW_SPEED_LIMITER),
                                  rotationRateLimiter = new SlewRateLimiter(Constants.SLEW_ROTATION_LIMITER);
    @Log
    private double slewSpeedNum;
    @Log
    private double slewRotationNum;

    private final DifferentialDriveOdometry m_odometry;

    private final Encoder m_leftEncoder =
      new Encoder(Constants.kLeftEncoderPorts[0], Constants.kLeftEncoderPorts[1], Constants.kEncoderReversed);
    private final Encoder m_rightEncoder =
      new Encoder(Constants.kRightEncoderPorts[0], Constants.kRightEncoderPorts[1], Constants.kEncoderReversed);

  /**
   * Creates a new DriveSubsystem.
   * 
   * Speed Multiplier starts at 1.0
   */
  public DriveSubsystem() {
    resetSpeedMultiplier();
    m_leftEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

  @Config.NumberSlider(defaultValue = Constants.SLEW_SPEED_LIMITER, min = 2, max = 6)
  public void setSpeedSlewRateNum(double speedRate) {
    speedRateLimiter = new SlewRateLimiter(speedRate);
    slewSpeedNum = speedRate;
  }

  @Config.NumberSlider(defaultValue = Constants.SLEW_ROTATION_LIMITER, min = 2, max = 6)
  public void setRotationSlewRateNum(double rotationRate) {
    rotationRateLimiter = new SlewRateLimiter(rotationRate);
    slewRotationNum = rotationRate;
  }

  public void setSpeedMultiplier(double speed) {
    speedMultiplier = speed;
  }

  public double getSpeedMultiplier() {
    return speedMultiplier;
  }

  public void setMaxOutput(double max) {
    drive.setMaxOutput(max);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    left.setVoltage(leftVolts);
    right.setVoltage(-rightVolts);
    drive.feed();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public void driveStraight() {
    double error = heading - gyro.getAngle();

    // Drives forward continuously at half speed, using the gyro to stabilize the heading
    drive.tankDrive(.5 + kP * error, .5 - kP * error);
  }

  public void tankDrive(double speedLeft, double speedRight) {
    drive.tankDrive(speedLeft, speedRight);
  }

  public void turnToAngle(int prospectiveAngle){
    double error = prospectiveAngle - gyro.getAngle();
    drive.tankDrive(kP * error, -kP * error);
  }

  public void resetSpeedMultiplier() { 
    // Defaults speedMultiplier to 1.0
    speedMultiplier = 1.0;
  }

  public void teleOpDrive(double straightSpeed, double turnSpeed) {
    // Drives at a forward speed and rotational speed
    drive.arcadeDrive(speedRateLimiter.calculate(straightSpeed * speedMultiplier), 
                      rotationRateLimiter.calculate(turnSpeed * Constants.SLOW_TURN_MULTIPLE * speedMultiplier));
    //logger.warning("Speed: " + straightSpeed + "SpeedMultiplier: " + speedMultiplier);
    // drive.arcadeDrive(straightSpeed * speedMultiplier, turnSpeed * Constants.SLOW_TURN_MULTIPLE * speedMultiplier);
    // The slow turn multiple makes the turning too slow at half speed, so we have commented it out for now.
  }

  public void turnDriveAtSpeed(double speed) {
    l1.set(ControlMode.PercentOutput, speed);
    l2.set(ControlMode.PercentOutput, speed);
    r1.set(ControlMode.PercentOutput, speed);
    r2.set(ControlMode.PercentOutput, speed);
  }

  public void straightDrive(double speed) {
    l1.set(ControlMode.PercentOutput, -speed);
    l2.set(ControlMode.PercentOutput, -speed);
    r1.set(ControlMode.PercentOutput, speed);
    r2.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler 
  }
}
