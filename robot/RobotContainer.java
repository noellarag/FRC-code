// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.BalanceCmd;
import frc.robot.commands.ElevatorMovementCmd;
import frc.robot.commands.EmergencyStopCmd;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.ArmCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

import java.io.IOException;
import java.nio.file.Path;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final XboxController m_xboxController = new XboxController(0);
  private final Elevator m_elevator = new Elevator();
    
  SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_armSubsystem.setDefaultCommand(new ArmCmd(m_armSubsystem, m_xboxController));
    m_drivetrain.setDefaultCommand(new ArcadeDriveCmd(m_drivetrain, m_xboxController));
    m_elevator.setDefaultCommand(new ElevatorMovementCmd(m_elevator, m_xboxController));
    
    // Configure the trigger bindings
    configureBindings();

    chooser.addOption("Straight 1 meter", loadTrajectoryToRamseteCommand(
        "/home/lvuser/deploy/pathplanner/generatedJSON/Straight1meter.wpilib.json",
        true));
    chooser.addOption("Straight and back", loadTrajectoryToRamseteCommand(
        "/home/lvuser/deploy/pathplanner/generatedJSON/StraightAndBack.wpilib.json",
        true));

    Shuffleboard.getTab("Autonomous").add(chooser);

  }

  public Command loadTrajectoryToRamseteCommand(String filename, boolean resetOdometry) {
    m_drivetrain.resetOdometry(new Pose2d());
    m_drivetrain.setSMARTLimit();
    Trajectory trajectory;
  
    try {
      
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }
    catch(IOException exception) {
      DriverStation.reportError("Unable to open trajectory" + filename, exception.getStackTrace());
      System.out.println("Unable to open trajectory" + filename);
      return new InstantCommand();
    }

    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_drivetrain::getPose, new RamseteController(DrivetrainConstants.kRamseteB, DrivetrainConstants.kRamsetZeta), new SimpleMotorFeedforward(DrivetrainConstants.KsVolts, DrivetrainConstants.ksVoltSecondsPerMeter, DrivetrainConstants.ksVoltSecondsSquaredperMeter), DrivetrainConstants.kDriveKinematics, m_drivetrain::getWheelSpeeds, new PIDController(DrivetrainConstants.kPDriveVel, 0 , 0), new PIDController(DrivetrainConstants.kPDriveVel, 0, 0), m_drivetrain::tankDriveVolts, m_drivetrain);

    if (resetOdometry) {
      return new SequentialCommandGroup(new InstantCommand(() -> m_drivetrain.resetOdometry(trajectory.getInitialPose())), ramseteCommand);
    } 
    else {
      return ramseteCommand;
    }
    

    
  }


  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return new SequentialCommandGroup(new ParallelCommandGroup(chooser.getSelected(), new EmergencyStopCmd(m_drivetrain)), new BalanceCmd(m_drivetrain));
  }
}