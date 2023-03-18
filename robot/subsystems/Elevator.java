// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final CANSparkMax m_elevatorMotor = new CANSparkMax(3, MotorType.kBrushed);
  private final Encoder m_elevatorEncoder = new Encoder(0,1,false,Encoder.EncodingType.k4X);
  /** Creates a new Elevator. */
  public Elevator() {
    m_elevatorMotor.setInverted(true);
    
    SmartDashboard.putNumber("Elevator encoder", Units.inchesToMeters(m_elevatorEncoder.getDistancePerPulse()));
    double distanceMeters = m_elevatorEncoder.getDistance();
    SmartDashboard.putNumber("Encoder Distance (m)", distanceMeters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void move(double speed) {
    m_elevatorMotor.set(speed);
  }

  public void setLimit(double limit) {
    if (m_elevatorEncoder.getDistance() > limit) {
      m_elevatorMotor.stopMotor();
    }
  }

  public void reset() {
    if (m_elevatorEncoder.getDistance() > 0.03) {
      m_elevatorMotor.set(-0.2);
    }
  }
  public void stop() {
    m_elevatorMotor.stopMotor();
  }
}
