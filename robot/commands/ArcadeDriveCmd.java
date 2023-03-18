// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDriveCmd extends CommandBase {

  Drivetrain m_drivetrain;
  private XboxController m_xboxController = new XboxController(0);
  /** Creates a new ArcadeDriveCmd. */
  public ArcadeDriveCmd(Drivetrain m_drivetrain, XboxController m_xboxController) {

    this.m_drivetrain = m_drivetrain;
    this.m_xboxController = m_xboxController;

    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setBreakMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drive(m_xboxController.getRightTriggerAxis(), m_xboxController.getLeftX());
    m_drivetrain.limitSpeed(1, 0.5, 0.2, m_xboxController.getLeftStickButtonReleased());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
