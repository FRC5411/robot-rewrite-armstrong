
package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotStates.DriveStates;
import frc.robot.Subsystems.DriveSubsystem;

public class ArcadeCommand extends CommandBase {

  private DoubleSupplier yInput;
  private DoubleSupplier xInput;

  private DriveSubsystem robotDrive;

  public ArcadeCommand(DoubleSupplier yInput, DoubleSupplier xInput, DriveSubsystem robotDrive) {
    this.yInput = yInput;
    this.xInput = xInput;

    this.robotDrive = robotDrive;

    addRequirements(robotDrive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double ySpeed = yInput.getAsDouble();
    double xRotation = xInput.getAsDouble();

    if (Math.abs(ySpeed) < DriveStates.sDeadzones) { ySpeed = 0.0; }
    if (Math.abs(xRotation) < DriveStates.sDeadzones) { xRotation = 0.0; }

    if (DriveStates.sIsSniperMode) {
      ySpeed *= DriveConstants.kSniperSpeedFactor;
      xRotation *= DriveConstants.kSniperSpeedFactor;
    }

    robotDrive.arcadeDrive(ySpeed, xRotation, DriveStates.sSquareInputs);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
