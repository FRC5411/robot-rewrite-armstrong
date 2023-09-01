// In Java We Trust

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ArcadeCommand;
import frc.robot.Constants.ControllerConstants;
import frc.robot.RobotStates.DriveStates;
import frc.robot.Subsystems.DriveSubsystem;

public class RobotContainer {

  private DriveSubsystem m_robotDrive;

  private CommandXboxController m_controller;

  private SendableChooser<Command> m_driverChooser;

  public RobotContainer() {
    m_robotDrive = new DriveSubsystem();

    m_controller = new CommandXboxController(ControllerConstants.kControllerPort);

    m_driverChooser = new SendableChooser<>();
    Shuffleboard.getTab("Driver Chooser").add(m_driverChooser);

    m_robotDrive.setDefaultCommand(new ArcadeCommand(
      () -> -m_controller.getLeftY(), 
      () -> -m_controller.getRightX(), 
      m_robotDrive));

    configureBindings();
    configureDriver();
  }

  private void configureBindings() {
    m_controller.leftTrigger().whileTrue(new InstantCommand( () -> {
      DriveStates.sIsSniperMode = true;
    }));
    m_controller.rightTrigger().whileTrue(new InstantCommand( () -> {
      DriveStates.sIsSniperMode = false;
    }));

    m_controller.y().onTrue(new InstantCommand( () -> {
      m_robotDrive.resetOdometry(new Pose2d());
    }));
  }

  private void configureDriver() {
    m_driverChooser.addOption("Aaron", new InstantCommand( () -> {
      DriveStates.sDeadzones = 0.1;
      DriveStates.sSquareInputs = true;
    }));
    m_driverChooser.addOption("Jack D", new InstantCommand( () -> {
      DriveStates.sDeadzones = 0.6;
      DriveStates.sSquareInputs = false;
    }));
    m_driverChooser.setDefaultOption("Default", new InstantCommand( () -> {
      DriveStates.sDeadzones = 0.0;
      DriveStates.sSquareInputs = false;
    }));
  }

  public Command getDriverProfile() {
    return m_driverChooser.getSelected();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
