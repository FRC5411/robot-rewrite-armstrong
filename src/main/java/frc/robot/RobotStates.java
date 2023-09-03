
package frc.robot;

public class RobotStates {

    public static class DriveStates {
        public static boolean sSquareInputs = false;
        public static boolean sIsSniperMode = false;
        public static double sDeadzones = 0.0;
    }

    public static class ArmStates {
        public static boolean sIsCone = false;
        public static boolean sShouldHoldArm = false;
        public static boolean sIsArmInSniper = false;
    }
}
