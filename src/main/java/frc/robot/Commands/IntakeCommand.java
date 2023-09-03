
package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotStates.ArmStates;
import frc.robot.Subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

  private IntakeSubsystem robotIntake;

  private boolean isIntaking;

  public IntakeCommand(IntakeSubsystem robotIntake, boolean isIntaking) {
    this.robotIntake = robotIntake;
    this.isIntaking = isIntaking;

    addRequirements(robotIntake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (isIntaking) {
      if (ArmStates.sIsCone) { robotIntake.setIntake(-0.5); }
      else { robotIntake.setIntake(1); }
    }
    else {
      if (ArmStates.sIsCone) { robotIntake.setIntake(0.5); }
      else { robotIntake.setIntake(-1); }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
