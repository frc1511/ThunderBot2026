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
            public static final int hangMotor = 2;
            public static final int lowerLimit = 0;
            public static final int upperLimit = 1;

        }
    }

    public interface HangConstants {
        double kZeroingSpeed = -0.1;
        double kMaxDeploySpeed = 0.2; // Extending Hanger
        double kMaxPullSpeed = -0.1;  // Retracting Hanger (pulling the robot up on the bar)
        double kMaxDeployDistanceRotations = 270;
        double kMaxPullDistanceRotations = 1; // This is not a delta and is absolute to the zero, DON'T confuse it for how much the robot is pulling down
    }
}
