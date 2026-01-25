package frc.util;

public class Constants {
    public class IOMap {
        /*
        *    +======================================+
        *    |                                      |
        *    |                                      |
        *    |   _____ _____  ___  ___ ___ ______   |
        *    |  |_   _|  _  | |  \/  |/ _ \| ___ \  |
        *    |    | | | | | | | .  . / /_\ | |_/ /  |
        *    |    | | | | | | | |\/| |  _  |  __/   |
        *    |   _| |_\ \_/ / | |  | | | | | |      |
        *    |   \___/ \___/  \_|  |_\_| |_\_|      |
        *    |                                      |
        *    |                                      |
        *    +======================================+
        */
        public class Hang {
            public static final int hangMotor = 67;
        }
    }

    public class HangConstants {
        public static final double kZeroingSpeed = -0.1;
        public static final double kMaxDelpoySpeed = 0.5; // Extending Hanger
        public static final double kMaxPullSpeed = -0.3;  // Retracting Hanger (pulling the robot up on the bar)
        public static final double kMaxDeployDistanceRotations = 10;
        public static final double kMaxPullDistanceRotations = 1; // This is not a delta and is absolute to the zero, DON'T confuse it for how much the robot is pulling down
    }
}
