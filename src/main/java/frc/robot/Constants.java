
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
    }

    public static class ControllerConstants {
        public static final int kControllerPort = 0;
    }
}
