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
import frc.robot.commands.SensorSlowCommand;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DistanceSensorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * 
 * 
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ControlPanelSubsystem m_controlPanelSubsystem = new ControlPanelSubsystem();
    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    private final LimeLightSubsystem m_limelightSubsystem = new LimeLightSubsystem();
    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    private final PneumaticsSubsystem m_pneumaticsSubsystem = new PneumaticsSubsystem();
    private final DistanceSensorSubsystem m_distanceSensorSubsystem = new DistanceSensorSubsystem();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

    private final XboxController m_functionsController = new XboxController(Constants.FUNCTIONS_CONTROLLER_PORT);
    private final XboxController m_driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);
    private final Runnable teleOpDriveFn = () -> m_driveSubsystem.teleOpDrive(-m_driverController.getY(Hand.kLeft), m_driverController.getX(Hand.kRight));

    private boolean givingBalls = true; //set at beginning
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // m_functionsController button uses
        whileHeldFuncController(Button.kB, m_pneumaticsSubsystem, m_pneumaticsSubsystem::extendControlPanelPiston);
        whileHeldFuncController(Button.kA, m_intakeSubsystem, m_intakeSubsystem::threeQuarterSpeed);
        whileHeldFuncController(Button.kBumperLeft, m_shooterSubsystem, m_shooterSubsystem::frontFullSpeed);
        whileHeldFuncController(Button.kBumperLeft, m_shooterSubsystem, m_shooterSubsystem::backFullSpeed);
        whileHeldFuncController(Button.kBumperRight, m_pneumaticsSubsystem, m_pneumaticsSubsystem::extendIntakePiston);
        
        // Driver Controller
        new JoystickButton(m_driverController, Button.kBumperRight.value)
            .whileHeld(new SensorSlowCommand(m_distanceSensorSubsystem, m_driveSubsystem, teleOpDriveFn));
            
        new JoystickButton(m_driverController, Button.kA.value)
            .whileHeld(new AutoAim(m_driveSubsystem, m_limelightSubsystem));
        //new JoystickButton(m_driverController, Button.kB.value)
        //    .toggleWhenPressed(new EncoderTest(m_controlPanelSubsystem));

        new JoystickButton(m_driverController, Button.kBumperLeft.value)
            .whenPressed(new InstantCommand(() -> m_driveSubsystem.setSpeedMultiplier(0.5), m_driveSubsystem));

        new JoystickButton(m_driverController, Button.kBumperLeft.value)
            .whenReleased(new InstantCommand(() -> m_driveSubsystem.setSpeedMultiplier(1.0), m_driveSubsystem));

        // Defaults
        m_controlPanelSubsystem.setDefaultCommand(new RunCommand(() -> m_controlPanelSubsystem.zeroSpeed(), m_controlPanelSubsystem));
        m_shooterSubsystem.setDefaultCommand(new RunCommand(() -> m_shooterSubsystem.zeroSpeed(), m_shooterSubsystem));
        m_intakeSubsystem.setDefaultCommand(new RunCommand(() -> m_intakeSubsystem.zeroSpeed(), m_intakeSubsystem));
        //m_driveSubsystem.setDefaultCommand(new RunCommand(teleOpDriveFn, m_driveSubsystem));
        m_driveSubsystem.setDefaultCommand(new RunCommand(() -> m_driveSubsystem.driveStraight(), m_driveSubsystem));
        m_limelightSubsystem.setDefaultCommand(new RunCommand(() -> m_limelightSubsystem.defaultReadings(), m_limelightSubsystem));
        m_pneumaticsSubsystem.setDefaultCommand(new RunCommand(() -> m_pneumaticsSubsystem.retractIntakePiston(), m_pneumaticsSubsystem));
        m_pneumaticsSubsystem.setDefaultCommand(new RunCommand(() -> m_pneumaticsSubsystem.retractControlPanelPiston(), m_pneumaticsSubsystem));
    }

    private void whileHeldFuncController(Button button, Subsystem subsystem, Runnable runnable) {
        new JoystickButton(m_functionsController, button.value).whileHeld(new InstantCommand(runnable, subsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     * 
     *         public Command getAutonomousCommand() { // An ExampleCommand will run
     *         in autonomous
     * 
     *         }
     */

    public Command getAutonomousCommand() { 
        // An ExampleCommand will run in autonomous

        return new SequentialCommandGroup(
            new AutoAim(m_driveSubsystem, m_limelightSubsystem),
            new RunCommand(() -> m_driveSubsystem.driveStraight(), m_driveSubsystem).withTimeout(1),
            new ParallelCommandGroup(
                new RunCommand(() -> m_shooterSubsystem.frontFullSpeed(), m_shooterSubsystem).withTimeout(3),
                new RunCommand(() -> m_shooterSubsystem.backThreeQuarterSpeed(), m_shooterSubsystem).withTimeout(3)));
    }
}
