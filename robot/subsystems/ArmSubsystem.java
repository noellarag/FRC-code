// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private final Spark intake = new Spark(9);
  private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  boolean armOpenedOnce;
  boolean buttonPressed;
  
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armOpenedOnce = false;
    buttonPressed = false;
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  public void start(boolean solenoidButton, boolean inButton, boolean outButton, double velocity) {
    if (solenoidButton) {
      
      if (armOpenedOnce) {
        m_solenoid.set(false);
        armOpenedOnce = false;
      }
      else {
        m_solenoid.set(true);
        armOpenedOnce = true;
      }
    }

    if(outButton){
      intake.set(velocity);
      buttonPressed = true;
    }
    else if (outButton) {
      intake.set(-velocity);
      buttonPressed = false;
    }
    else {
      intake.set(0);
      buttonPressed = false;
    }
  }

  public void reset() {
    m_solenoid.set(false);
    intake.set(0);
  }
}
