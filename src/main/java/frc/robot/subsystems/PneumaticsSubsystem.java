/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticsSubsystem extends SubsystemBase {
    /**
     * Creates a new IntakeSubsytem.
     */
    DoubleSolenoid solenoidI = new DoubleSolenoid(Constants.INTAKE_FORWARD_CHANNEL, Constants.INTAKE_REVERSE_CHANNEL);
    DoubleSolenoid solenoidCP = new DoubleSolenoid(Constants.CONTROLPANEL_FORWARD_CHANNEL, Constants.CONTROLPANEL_REVERSE_CHANNEL);
    
    public PneumaticsSubsystem() {
        
    }
  
    public void extendIntakePiston() {
        solenoidI.set(kForward);
    }

    public void retractIntakePiston() {
        solenoidI.set(kReverse);
    }

    public void extendControlPanelPiston() {
      solenoidCP.set(kForward);
  }

  public void retractControlPanel() {
      solenoidCP.set(kReverse);
  }

    public void closeValves() {
      solenoidI.set(kOff);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  }