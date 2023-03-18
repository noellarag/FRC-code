package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorMovementCmd extends CommandBase {
  private final Elevator m_elevator;
  XboxController m_xboxController = new XboxController(0);
  /** Creates a new ElevatorMovementCmd. */
  public ElevatorMovementCmd(Elevator m_elevator, XboxController m_xboxController) {
    this.m_elevator = m_elevator;
    this.m_xboxController = m_xboxController;
    addRequirements(m_elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_xboxController.getRightY() > 0.1 || m_xboxController.getRightY() < -0.1) {
      m_elevator.move(m_xboxController.getRightY());
    }
    else if (m_xboxController.getRightY() < 0.1 || m_xboxController.getRightY() > -0.1) {
      m_elevator.move(0);
    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (DriverStation.isAutonomousEnabled()) {
        return false;
    }
    else {
        return true;
    }
  }
}
