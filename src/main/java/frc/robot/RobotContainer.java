/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.AutoAim;
import frc.robot.commands.ControlPanelPosition;
import frc.robot.commands.ControlPanelRotation;
import frc.robot.commands.ColorWheelApproach;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.DistanceSensorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import io.github.oblarg.oblog.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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

        Logger.configureLoggingAndConfig(this, false);
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
        whileHeldFuncController(Button.kBumperLeft, m_shooterSubsystem, m_shooterSubsystem::shootBalls);
        whileHeldFuncController(Button.kBumperRight, m_pneumaticsSubsystem, m_pneumaticsSubsystem::extendIntakePiston);

        new JoystickButton(m_functionsController, Button.kB.value)
            .whileHeld(new ColorWheelApproach(m_driveSubsystem, m_distanceSensorSubsystem));

        new JoystickButton(m_functionsController, Button.kX.value)
            .toggleWhenPressed(new ControlPanelPosition(m_controlPanelSubsystem));
        
        new JoystickButton(m_functionsController, Button.kY.value)
            .toggleWhenPressed(new ControlPanelRotation(m_controlPanelSubsystem));
        
        // Driver Controller
        //new JoystickButton(m_driverController, Button.kBumperLeft.value)
        //    .whileHeld(new SensorSlowCommand(m_distanceSensorSubsystem, m_driveSubsystem, teleOpDriveFn));

        new JoystickButton(m_driverController, Button.kB.value)
            .whileHeld(new ColorWheelApproach(m_driveSubsystem, m_distanceSensorSubsystem));

        new JoystickButton(m_driverController, Button.kBumperLeft.value)
            .whileHeld(new AutoAim(m_driveSubsystem, m_limelightSubsystem, m_distanceSensorSubsystem));
            
        new JoystickButton(m_driverController, Button.kA.value)
            .whileHeld(new AutoAim(m_driveSubsystem, m_limelightSubsystem, m_distanceSensorSubsystem));

        new JoystickButton(m_driverController, Button.kBumperRight.value)
            .whenPressed(() -> m_driveSubsystem.setMaxOutput(0.6))
            .whenReleased(() -> m_driveSubsystem.setMaxOutput(1));

        //new JoystickButton(m_driverController, Button.kBumperLeft.value)
        //    .whenPressed(() ->     NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0))
        //    .whenReleased(() ->    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1));

        


        // Defaults
        m_controlPanelSubsystem.setDefaultCommand(new RunCommand(() -> m_controlPanelSubsystem.zeroSpeed(), m_controlPanelSubsystem));
        m_shooterSubsystem.setDefaultCommand(new RunCommand(() -> m_shooterSubsystem.zeroSpeed(), m_shooterSubsystem));
        m_intakeSubsystem.setDefaultCommand(new RunCommand(() -> m_intakeSubsystem.zeroSpeed(), m_intakeSubsystem));
        m_driveSubsystem.setDefaultCommand(new RunCommand(teleOpDriveFn, m_driveSubsystem));
        //m_driveSubsystem.setDefaultCommand(new RunCommand(() -> m_driveSubsystem.driveStraight(), m_driveSubsystem));
        m_limelightSubsystem.setDefaultCommand(new RunCommand(() -> m_limelightSubsystem.lightOn(), m_limelightSubsystem));
        m_pneumaticsSubsystem.setDefaultCommand(new RunCommand(() -> m_pneumaticsSubsystem.retractBothPistons(), m_pneumaticsSubsystem));
        SmartDashboard.putData("Blue", new InstantCommand(() -> m_controlPanelSubsystem.colorRotation("B")));
        SmartDashboard.putData("Red", new InstantCommand(() -> m_controlPanelSubsystem.colorRotation("R")));
        SmartDashboard.putData("Green", new InstantCommand(() -> m_controlPanelSubsystem.colorRotation("G")));
        SmartDashboard.putData("Yellow", new InstantCommand(() -> m_controlPanelSubsystem.colorRotation("Y")));

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
        
        // Trajectory Control
        
        var  autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter),
                                       Constants.kDriveKinematics, 10);

        // Create config for trajectory
        TrajectoryConfig backwardConfig =
            new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                                Constants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Constants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint)
                .setReversed(true);

        TrajectoryConfig forwardConfig =
                new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                                    Constants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint)
                    .setReversed(false);
                
        Trajectory getBallsFromTrenchTrajectory = TrajectoryGenerator.generateTrajectory(
                    // Start at the origin facing the +X direction
                    new Pose2d(0, 0, new Rotation2d(0, 0)),
                    List.of(),
                    new Pose2d(-0.5, 0, new Rotation2d(0, 0)), // -1.6, 0
                    // Pass config
                    backwardConfig);
        
        Trajectory trenchToTurnTrajecory = TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0, 0)),
                        List.of(),
                        new Pose2d(0.3, 0, new Rotation2d(0, 5)),//2.8 , 0/7
                        // Pass config
                        forwardConfig);

        Trajectory driveToPortTrajecory = TrajectoryGenerator.generateTrajectory(
                            // Start at the origin facing the +X direction
                            new Pose2d(0, 0, new Rotation2d(0, 0)),
                            List.of(),
                            new Pose2d(1.4, 0, new Rotation2d(0, 0)),//2.8 , 0/7
                            // Pass config
                            forwardConfig);
        
        RamseteCommand collectFromTrenchCommand = new RamseteCommand(
                        getBallsFromTrenchTrajectory,
                        m_driveSubsystem::getPose,
                        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                        new SimpleMotorFeedforward(Constants.ksVolts,
                                                   Constants.kvVoltSecondsPerMeter,
                                                   Constants.kaVoltSecondsSquaredPerMeter),
                                                    Constants.kDriveKinematics,
                        m_driveSubsystem::getWheelSpeeds,
                        new PIDController(Constants.kPDriveVel, 0, 0),
                        new PIDController(Constants.kPDriveVel, 0, 0),
                        // RamseteCommand passes volts to the callback
                        m_driveSubsystem::tankDriveVolts,
                        m_driveSubsystem
                    );        
        RamseteCommand trenchToTurnCommand = new RamseteCommand(
                        trenchToTurnTrajecory,
                        m_driveSubsystem::getPose,
                        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                        new SimpleMotorFeedforward(Constants.ksVolts,
                                                   Constants.kvVoltSecondsPerMeter,
                                                   Constants.kaVoltSecondsSquaredPerMeter),
                                                    Constants.kDriveKinematics,
                        m_driveSubsystem::getWheelSpeeds,
                        new PIDController(Constants.kPDriveVel, 0, 0),
                        new PIDController(Constants.kPDriveVel, 0, 0),
                        // RamseteCommand passes volts to the callback
                        m_driveSubsystem::tankDriveVolts,
                        m_driveSubsystem
                    );
        
        RamseteCommand driveToPortCommand = new RamseteCommand(
                        driveToPortTrajecory,
                        m_driveSubsystem::getPose,
                        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                        new SimpleMotorFeedforward(Constants.ksVolts,
                                                   Constants.kvVoltSecondsPerMeter,
                                                   Constants.kaVoltSecondsSquaredPerMeter),
                                                    Constants.kDriveKinematics,
                        m_driveSubsystem::getWheelSpeeds,
                        new PIDController(Constants.kPDriveVel, 0, 0),
                        new PIDController(Constants.kPDriveVel, 0, 0),
                        // RamseteCommand passes volts to the callback
                        m_driveSubsystem::tankDriveVolts,
                        m_driveSubsystem
                    );

        //return straightCommand.andThen(() -> m_driveSubsystem.tankDriveVolts(0, 0));
        
        return new SequentialCommandGroup(
            new RunCommand(() -> m_pneumaticsSubsystem.extendIntakePiston(), m_pneumaticsSubsystem).withTimeout(0.1),
            new RunCommand(() -> m_intakeSubsystem.threeQuarterSpeed(), m_intakeSubsystem).withTimeout(0.1),
            collectFromTrenchCommand.andThen(() -> m_driveSubsystem.tankDriveVolts(0, 0)), 
            new RunCommand(() -> m_pneumaticsSubsystem.retractIntakePiston(), m_pneumaticsSubsystem).withTimeout(0.1),
            new RunCommand(() -> m_intakeSubsystem.zeroSpeed(), m_intakeSubsystem).withTimeout(0.1),
            driveToPortCommand.andThen(() -> m_driveSubsystem.tankDriveVolts(0, 0)), 
            new RunCommand(() -> m_shooterSubsystem.shootBalls(), m_shooterSubsystem).withTimeout(2.0),
            new RunCommand(() -> m_shooterSubsystem.zeroSpeed(), m_shooterSubsystem).withTimeout(0.1));
        
    }
}
