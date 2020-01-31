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

public class PneumaticsSubsystem extends SubsystemBase {
    /**
     * Creates a new IntakeSubsytem.
     */
    DoubleSolenoid solenoid1 = new DoubleSolenoid(0, 1);
    
    public PneumaticsSubsystem() {
        
    }
  
    public void extendPiston() {
        solenoid1.set(kForward);
    }

    public void retractPiston() {
        solenoid1.set(kReverse);
    }

    public void closeValves() {
      solenoid1.set(kOff);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  }