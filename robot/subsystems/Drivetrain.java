// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {

  private final CANSparkMax m_leftDrive = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax m_rightDrive = new CANSparkMax(2, MotorType.kBrushless);

  private final RelativeEncoder m_leftEncoder = m_leftDrive.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightDrive.getEncoder();

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_rightDrive, m_leftDrive);
  private final XboxController m_xboxController = new XboxController(0);

  private final static AHRS navx = new AHRS(SerialPort.Port.kMXP);
  private final DifferentialDriveOdometry m_odometry;

  boolean rBPressed;
  boolean rBPressedTwice;

  public Drivetrain() {
    m_rightDrive.setInverted(true);
    rBPressed = false;
    rBPressedTwice = false;
    
    m_leftEncoder.setPositionConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor);
    m_rightEncoder.setPositionConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor);
    m_leftEncoder.setVelocityConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor / 60);
    m_rightEncoder.setVelocityConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor / 60);

    m_odometry = new DifferentialDriveOdometry(navx.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    m_odometry.resetPosition(navx.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), new Pose2d());
    navx.reset();
    navx.calibrate();
    resetEncoders();

    m_leftDrive.setSmartCurrentLimit(50, 60, 100);
    m_rightDrive.setSmartCurrentLimit(50,60,100);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void drive(double ySpeed, double xRotation) {
    m_robotDrive.arcadeDrive(ySpeed, -xRotation);
    if (m_xboxController.getLeftTriggerAxis() > 0.100) {
      m_robotDrive.arcadeDrive(-m_xboxController.getLeftTriggerAxis(), -xRotation);
    }
    
  }
  public void limitSpeed(double maxSpeedLimit, double mediumSpeedLimit, double lowerSpeedLimit, boolean velocityButton) {
    if (velocityButton) {
      if (rBPressedTwice) {
        m_robotDrive.setMaxOutput(maxSpeedLimit);
        rBPressedTwice = false;
      }
      else {
        if (rBPressed) {
          m_robotDrive.setMaxOutput(mediumSpeedLimit);
          rBPressed = false;
          rBPressedTwice = true;
        }
        else {
          m_robotDrive.setMaxOutput(lowerSpeedLimit);
          rBPressed = true;
        }
      }
    }  
  }

  public void setSMARTLimit() {
    m_leftDrive.setSmartCurrentLimit(100, 100, 100);
    m_rightDrive.setSmartCurrentLimit(100,100,100);
  }

  public void setBreakMode() {
    m_leftDrive.setIdleMode(IdleMode.kBrake);
    m_rightDrive.setIdleMode(IdleMode.kBrake);
  }
  
  public void setCoastMode() {
    m_leftDrive.setIdleMode(IdleMode.kCoast);
    m_rightDrive.setIdleMode(IdleMode.kCoast);
  }

  public double getRightEncoderPosition() {
    return m_rightEncoder.getPosition();
  }

  public double getleftEncoderPosition() {
    return -m_leftEncoder.getPosition();
  }


  public double getRightEncoderVelocity() {
    return m_rightEncoder.getVelocity();
  }

  public double getLeftEncoderVelocity() {
    return m_leftEncoder.getVelocity();
  }

  public void resetEncoders() {
    m_rightEncoder.setPosition(0);
    m_leftEncoder.setPosition(0);
  }

  public double getTurnRate() {
    return -navx.getRate();
  }

  public static double getHeading() {
    return navx.getRotation2d().getDegrees();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    //m_odometry.resetPosition(navx.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
    m_odometry.resetPosition(navx.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }
  
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftDrive.setVoltage(leftVolts);
    m_rightDrive.setVoltage(rightVolts);
    m_robotDrive.feed();
  }

  public double getAverageEncoderDistance() {
    return (getleftEncoderPosition() + getRightEncoderPosition() / 2);
  }

  public RelativeEncoder getLeftEncoder() {
    return m_leftEncoder;
  }
  
  public RelativeEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  public void setMaxOutput(double maxOutput) {
    m_robotDrive.setMaxOutput(maxOutput);
  }

  public static void zeroHeading() {
    navx.reset();
    navx.calibrate();
  }

  public Gyro getGyro() {
    return getGyro();
  }

  public void emergencyStop() {
    m_robotDrive.stopMotor();
    m_leftDrive.set(0);
    m_rightDrive.set(0);
  }

  public double motorSpeed() {
    return ((getLeftEncoderVelocity() + getRightEncoderVelocity()) / 2);
  }
}
