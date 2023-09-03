
package frc.robot.Commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotStates.ArmStates;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Utils.ConsoleWriter;

public class ArmCommand extends ProfiledPIDCommand {

  public ArmCommand(ArmSubsystem robotArm, String goal, boolean isCone) {
    super(
        new ProfiledPIDController(
            0.066,
            0,
            0,
            new TrapezoidProfile.Constraints(ArmConstants.kArmVelocity, ArmConstants.kArmAccel)),
        // This should return the measurement
        () -> robotArm.getArmPosition(),
        // This should return the goal (can also be a constant)
        () -> getSetpoint(goal, isCone),
        // This uses the output
        (output, setpoint) -> {
          ArmStates.sShouldHoldArm = false;
          robotArm.setArm(output);
        }, 
        robotArm);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(robotArm);
  }

  public static int getSetpoint(String goal, boolean isCone) {
    switch (goal) {
      case "high":
        if (isCone) { return ArmConstants.kConeHigh;}
        else { return ArmConstants.kCubeHigh; }
      case "mid":
        if (isCone) { return ArmConstants.kConeMid;}
        else { return ArmConstants.kCubeMid; }
      case "low":
        if (isCone) { return ArmConstants.kConeLow;}
        else { return ArmConstants.kCubeLow; }
      case "substation":
        if (isCone) { return ArmConstants.kConeSub;}
        else { return ArmConstants.kCubeSub; }
      case "ground":
        if (isCone) { return ArmConstants.kConeGround;}
        else { return ArmConstants.kCubeGround; }
      case "idle":
        return ArmConstants.kIdle;
      default:
        ConsoleWriter.printError("Could not get setpoint", "/ArmCommand/");
        return 0;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
