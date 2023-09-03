
package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static class DriveConstants {
        public static final int kLeftFront = 11;
        public static final int kLeftBack = 12;
        public static final int kRightFront = 13;
        public static final int kRightBack = 14;

        public static final double kWheelRadiusMeters = Units.inchesToMeters(3);
        public static final double kTrackWidthMeters = Units.inchesToMeters(6);
        public static final double kGearRatio = 7.89;

        public static final double kLinearDistanceConversion = Units.inchesToMeters(1 / kGearRatio * 2 * Math.PI * kWheelRadiusMeters) * (2.16/0.548) * 10;

        public static final DifferentialDriveKinematics kTankKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

        public static final double kSniperSpeedFactor = 0.4;

        public static final double kBalanceVelocity = 1.0;
        public static final double kBalanceAccel = 0.5;
    }

    public static class ArmConstants {
        public static final int kArmMotor = 21;

        public static final double kArmEncoderCF = 22.755;

        public static final int kArmMotorCurrentLimit = 60;

        public static final int[] kArmEncoderDIOPorts = {0, 1};

        public static final int kArmVelocity = 250;
        public static final int kArmAccel = 100;

        public static final int kConeHigh = 172;
        public static final int kConeMid = 193;
        public static final int kConeLow = 112;
        public static final int kConeSub = 174;
        public static final int kConeGround = 257;

        public static final int kCubeHigh = 173;
        public static final int kCubeMid = 142;
        public static final int kCubeLow = 112;
        public static final int kCubeSub = 178;
        public static final int kCubeGround = 263;

        public static final int kIdle = 0;
        public static final double kArmFlat = 33.43;
    }

    public static class IntakeConstants {
        public static final int kIntakeMotor = 22;

        public static final int kIntakeMotorCurrentLimit = 50;
    }

    public static class ControllerConstants {
        public static final int kControllerPort = 0;
        public static final int kButtonBoardPort = 1;
    }
}
