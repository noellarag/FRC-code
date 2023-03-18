// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class BalanceCmd extends CommandBase {
  Drivetrain m_drivetrain;
  private AHRS navx = new AHRS(SerialPort.Port.kMXP);
  private Timer m_timer = new Timer();
  /** Creates a new BalanceCmd. */
  public BalanceCmd(Drivetrain m_drivetrain) {
    this.m_drivetrain = m_drivetrain;
    addRequirements(m_drivetrain);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetAngle = 0.0;
    double angleThreshold = 0.1;
    double angleThresholdB = 0.8;
    double maxTurnSpeed = 0.6;
    double currentAngle = navx.getAngle();
    double angleDifference = (((targetAngle - currentAngle) % 360) + 540) % 360 - 180;
    boolean hasMovedUnexpectedly = Math.abs(angleDifference) > angleThreshold;
    boolean hasMovedUnexpectedlyB = Math.abs(angleDifference) > angleThresholdB;
    double turnSpeed = Math.min(Math.abs(angleDifference) / 45.0, maxTurnSpeed);
        
    double zAngle = navx.getRoll(); // get the roll angle around the Z-axis
        
      if (hasMovedUnexpectedly) {
        m_timer.stop();

        if (zAngle < -4.0) { // if the robot is inclined forward
          m_drivetrain.drive(0.3, 0); 
        } 
        else if (zAngle > 4.0) { // if the robot is inclined backward
          m_drivetrain.drive(-0.3, 0); // move forward
        }

      }
      else if (hasMovedUnexpectedlyB) {
        m_timer.stop();
        if (angleDifference > 0) {
          m_drivetrain.drive(0.0, turnSpeed);
        } 
        else if (angleDifference < 0) {
          m_drivetrain.drive(0.0, -turnSpeed);
        }
      }
      else {
        m_drivetrain.drive(0, 0);
        m_timer.reset();
        m_timer.start();

      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.hasElapsed(3)) {
      return true;
    }
    else {
      return false;
    }
  }
}
