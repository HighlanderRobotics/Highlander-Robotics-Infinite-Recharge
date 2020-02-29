/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Characterization Constants (EDIT LATER)
        // Feedforward/feedback gains
    public static final double ksVolts = 0;
    public static final double kvVoltSecondsPerMeter = 0;
    public static final double kaVoltSecondsSquaredPerMeter = 0;
    public static final double kPDriveVel = 0;
        // Differential Drive Kinematics
    public static final double kTrackwidthMeters = 0;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
        // Max Trajectory Velocity/Acceleration
    public static final double kMaxSpeedMetersPerSecond = 0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0;
        //Ramsete
    public static double kRamseteB;
    public static double kRamseteZeta;
    


	// Controllers
    public static int FUNCTIONS_CONTROLLER_PORT = 0;
    public static int DRIVER_CONTROLLER_PORT = 1;

    // Driving multipliers
    public static double HALF_SPEED_MULTIPLIER = 0.5;
    public static double SLOW_TURN_MULTIPLE = 0.75;

    // Distance Sensor
    public static int PING_CHANNEL = 0;
    public static int ECHO_CHANNEL = 1;

    // Motors
    public static int DRIVESUBSYSTEM_LEFT_BACK_TALON = 0;
    public static int DRIVESUBSYSTEM_LEFT_FRONT_VICTOR = 1;
    public static int DRIVESUBSYSTEM_RIGHT_BACK_TALON = 3;
    public static int DRIVESUBSYSTEM_RIGHT_FRONT_VICTOR = 2;
    public static int CONTROLPANELSUBSYSTEM_VICTOR = 4;
    public static int SHOOTERSUBSYSTEM_BACK_VICTOR = 5;
    public static int SHOOTERSUBSYSTEM_FRONT_VICTOR = 6;
    public static int INTAKESUBSYSTEM_VICTOR = 7;

    // Solenoids
    public static final int INTAKE_FORWARD_CHANNEL = 4;
    public static final int INTAKE_REVERSE_CHANNEL = 5;
    public static final int CONTROLPANEL_FORWARD_CHANNEL = 7;
    public static final int CONTROLPANEL_REVERSE_CHANNEL = 6;
    
    // Limiters
    public static double SLEW_SPEED_LIMITER = 4;
    public static double SLEW_ROTATION_LIMITER = 3.5;

	public static boolean kGyroReversed = false;
	public static DigitalSource[] kLeftEncoderPorts;
	public static DigitalSource[] kRightEncoderPorts;
}
