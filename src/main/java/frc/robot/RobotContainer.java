// In Java We Trust

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ArcadeCommand;
import frc.robot.Commands.ArmCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Constants.ControllerConstants;
import frc.robot.RobotStates.ArmStates;
import frc.robot.RobotStates.DriveStates;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

public class RobotContainer {

  private DriveSubsystem m_robotDrive;
  private ArmSubsystem m_robotArm;
  private IntakeSubsystem m_robotIntake;

  private PowerDistribution m_PDH;

  private CommandXboxController m_controller;
  private CommandGenericHID m_buttonBoard;

  private SendableChooser<Command> m_driverChooser;

  public RobotContainer() {
    m_robotDrive = new DriveSubsystem();
    m_robotArm = new ArmSubsystem();
    m_robotIntake = new IntakeSubsystem();

    m_PDH = new PowerDistribution(ControllerConstants.kPDhPort, ModuleType.kRev);

    m_controller = new CommandXboxController(ControllerConstants.kControllerPort);
    m_buttonBoard = new CommandGenericHID(ControllerConstants.kButtonBoardPort);

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

    /* Button Board Bindings */

    // High
    m_buttonBoard.button(1)
    .whileTrue(new ArmCommand(m_robotArm,"high", ArmStates.sIsCone))
    .whileFalse(holdArm());

    // Mid
    m_buttonBoard.button(2)
    .whileTrue(new ArmCommand(m_robotArm,"mid", ArmStates.sIsCone))
    .whileFalse(holdArm());

    // Low
    m_buttonBoard.button(3)
    .whileTrue(new ArmCommand(m_robotArm,"low", ArmStates.sIsCone))
    .whileFalse(holdArm());

    // Substation
    m_buttonBoard.button(8)
    .whileTrue(new ArmCommand(m_robotArm,"substation", ArmStates.sIsCone))
    .whileFalse(holdArm());

    // Ground
    m_buttonBoard.button(7)
    .whileTrue(new ArmCommand(m_robotArm,"ground", ArmStates.sIsCone))
    .whileFalse(holdArm());

    // Idle
    m_buttonBoard.button(9)
      .whileTrue(new ArmCommand(m_robotArm,"idle", ArmStates.sIsCone))
      .whileFalse(holdArm());

    // Arm Out
    m_buttonBoard.button(10)
      .whileTrue(new InstantCommand( () -> { m_robotArm.setArm(0.5, ArmStates.sIsArmInSniper); }))
      .whileFalse(holdArm());

    // Arm In
    m_buttonBoard.button(11)
      .whileTrue(new InstantCommand( () -> { m_robotArm.setArm(-0.5, ArmStates.sIsArmInSniper); }))
      .whileFalse(holdArm());

    // Toggle Mode
    m_buttonBoard.button(5)
      .toggleOnTrue(new InstantCommand( () -> {
      ArmStates.sIsCone = true;
      m_PDH.setSwitchableChannel(true);
     }))
      .toggleOnFalse(new InstantCommand( () -> {
      ArmStates.sIsCone = false; 
      m_PDH.setSwitchableChannel(false);
    }));

    // Arm Sniper
    m_buttonBoard.button(11)
      .whileTrue(new InstantCommand( () -> { ArmStates.sIsArmInSniper = true; }))
      .whileFalse(new InstantCommand( () -> { ArmStates.sIsArmInSniper = false; }));

    // Intake In
    m_buttonBoard.button(12)
    .whileTrue(new IntakeCommand(m_robotIntake, true))
    .whileFalse(new InstantCommand( () -> { m_robotIntake.setIntake(0.0); }));
    
    // Intake Out
    m_buttonBoard.button(6)
      .whileTrue(new IntakeCommand(m_robotIntake, false))
      .whileFalse(new InstantCommand( () -> { m_robotIntake.setIntake(0.0); }));
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

  private Command holdArm() {
    return new InstantCommand( () -> {
      ArmStates.sShouldHoldArm = true;

      m_robotArm.setArm(0.0);
      m_robotArm.holdArm(m_robotArm).schedule();
    });
  }

  public Command getDriverProfile() {
    return m_driverChooser.getSelected();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
