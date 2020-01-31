/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.AutoAim;
import frc.robot.commands.ControlPanelPosition;
import frc.robot.commands.ControlPanelRotation;
import frc.robot.commands.SensorSlowCommand;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterSubsytem;
import frc.robot.subsystems.distanceSensorSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
 

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
// The robot's subsystems and commands are defined here...
  private final ControlPanelSubsystem m_controlPanelSubsystem = new ControlPanelSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final LimeLightSubsystem m_limelightSubsystem = new LimeLightSubsystem();
  private final ShooterSubsytem m_shooterSubsystem = new ShooterSubsytem();
  private final distanceSensorSubsystem m_DistanceSensorSubsystem = new distanceSensorSubsystem();

  private final XboxController m_functionsController = new XboxController(Constants.FUNCTIONS_CONTROLLER_PORT);
  private final XboxController m_driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // m_functionsController button uses
    new JoystickButton(m_functionsController, Button.kBumperRight.value)
        .whileHeld(new RunCommand(() -> m_shooterSubsystem.backQuarterSpeed(), m_shooterSubsystem).withTimeout(0.5));
    new JoystickButton(m_functionsController, Button.kB.value)
        .whileHeld(new InstantCommand(() -> m_shooterSubsystem.backHalfSpeed(), m_shooterSubsystem));
    new JoystickButton(m_functionsController, Button.kY.value)
        .whileHeld(new InstantCommand(() -> m_shooterSubsystem.backThreeQuarterSpeed(), m_shooterSubsystem));
    new JoystickButton(m_functionsController, Button.kX.value)
        .whileHeld(new InstantCommand(() -> m_shooterSubsystem.backFullSpeed(), m_shooterSubsystem));
    new JoystickButton(m_functionsController, Button.kA.value)
        .whileHeld(new InstantCommand(() -> m_shooterSubsystem.frontQuarterSpeed(), m_shooterSubsystem));
    new JoystickButton(m_functionsController, Button.kB.value)
        .whileHeld(new InstantCommand(() -> m_shooterSubsystem.frontHalfSpeed(), m_shooterSubsystem));
    new JoystickButton(m_functionsController, Button.kY.value)
        .whileHeld(new InstantCommand(() -> m_shooterSubsystem.frontThreeQuarterSpeed(), m_shooterSubsystem));
    new JoystickButton(m_functionsController, Button.kX.value)
        .whileHeld(new InstantCommand(() -> m_shooterSubsystem.frontFullSpeed(), m_shooterSubsystem));
    
    whileHeldFuncController(Button.kX, m_shooterSubsystem, m_shooterSubsystem::frontFullSpeed);
    whileHeldFuncController(Button.kBumperLeft, m_shooterSubsystem, m_shooterSubsystem::frontFullSpeed);
    whileHeldFuncController(Button.kBumperLeft, m_shooterSubsystem, m_shooterSubsystem::backFullSpeed);

    new JoystickButton(m_driverController, Button.kBumperRight.value)
        .whileHeld(new SensorSlowCommand(m_DistanceSensorSubsystem, m_driveSubsystem));
    
    // m_driverController button uses
    new JoystickButton(m_driverController, Button.kBumperLeft.value)
        .whileHeld(new InstantCommand(() -> m_driveSubsystem.teleOpDriveHalfSpeed(m_driverController), m_driveSubsystem));

    m_controlPanelSubsystem.setDefaultCommand(new RunCommand(() -> m_controlPanelSubsystem.zeroSpeed(), m_controlPanelSubsystem));
    m_shooterSubsystem.setDefaultCommand(new RunCommand(() -> m_shooterSubsystem.zeroSpeed(), m_shooterSubsystem));
    m_driveSubsystem.setDefaultCommand(new RunCommand(() -> m_driveSubsystem.teleOpDrive(m_driverController), m_driveSubsystem));
    m_limelightSubsystem.setDefaultCommand(new RunCommand(() -> m_limelightSubsystem.defaultReadings(), m_limelightSubsystem));
    }

    private void whileHeldFuncController(Button button, Subsystem subsystem, Runnable runnable) {
        new JoystickButton(m_functionsController, button.value)
        .whileHeld(new InstantCommand(runnable, subsystem)); 
    }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    
  }
  */
}
