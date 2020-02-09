/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Controllers
    public static int FUNCTIONS_CONTROLLER_PORT = 0;
    public static int DRIVER_CONTROLLER_PORT = 1;

    // Driving multipliers
    public static double HALF_SPEED_MULTIPLIER = 0.5;
    public static double SLOW_TURN_MULTIPLE = 0.85;

    // Distance Sensor
    public static int PING_CHANNEL = 0;
    public static int ECHO_CHANNEL = 1;

    // Motors
    public static int DRIVESUBSYSTEM_RIGHT_FRONT_VICTOR = 0;
    public static int DRIVESUBSYSTEM_RIGHT_BACK_TALON = 1;
    public static int DRIVESUBSYSTEM_LEFT_BACK_TALON = 2;
    public static int DRIVESUBSYSTEM_LEFT_FRONT_VICTOR = 3;
    public static int CONTROLPANELSUBSYSTEM_VICTOR = 4;
    public static int SHOOTERSUBSYSTEM_BACK_VICTOR = 5;
    public static int SHOOTERSUBSYSTEM_FRONT_VICTOR = 6;
    public static int INTAKESUBSYSTEM_VICTOR = 7;
}
