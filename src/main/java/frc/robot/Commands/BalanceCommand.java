
package frc.robot.Commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.DriveSubsystem;

public class BalanceCommand extends ProfiledPIDCommand {

  public BalanceCommand(DriveSubsystem robotDrive) {
    super(
        new ProfiledPIDController(
            0.0243,
            0,
            0,
            new TrapezoidProfile.Constraints(DriveConstants.kBalanceVelocity, 
              DriveConstants.kBalanceAccel)),
        // This should return the measurement
        () -> robotDrive.getPitch(),
        // This should return the goal (can also be a constant)
        () -> 0.0,
        // This uses the output
        (output, setpoint) -> {
          robotDrive.arcadeDrive(output, 0.0, false);
        },
        robotDrive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(2.5);
    addRequirements(robotDrive);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
