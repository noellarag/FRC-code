// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCmd extends CommandBase {
  private ArmSubsystem m_arm;
  private XboxController m_xboxController = new XboxController(0);
  /** Creates a new ArmCmd. */
  public ArmCmd(ArmSubsystem m_arm, XboxController m_xboxController) {
    this.m_arm = m_arm;
    this.m_xboxController = m_xboxController;
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.start(m_xboxController.getAButtonReleased(), m_xboxController.getLeftBumper(), m_xboxController.getRightBumper(), 0.7);
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
