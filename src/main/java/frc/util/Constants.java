package frc.util;

public class Constants {
    public static final double kAntiSpamAlertTimeout = 5; // in seconds

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
        double kSetpointMaxVelocity = 0.02; // Prevents flybys
        double kSetpointPositionTolerance = 0.05;
        double kZeroingSpeed = -0.1;
        double kMaxDeploySpeed = 0.4; // TODO: DOUBLE FOR LATER TESTING // Extending Hanger
        double kMaxPullSpeed = -0.3;  // TODO: DOUBLE FOR LATER TESTING // Retracting Hanger (pulling the robot up on the bar)
        double kMaxDeployDistanceRotations = 33;
        double kMaxPullDistanceRotations = 8; // This is not a delta and is absolute to the zero, DON'T confuse it for how much the robot is pulling down
    }
}
