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

        public interface Shooter {
            //TODO: actual can ids
            int shooterMotor = 20;
            int hoodMotor = 21;
            int turretMotor = 22;
        }
    }

    protected interface BasePID {
        double kP = 0.0d;
        double kI = 0.0d;
        double kD = 0.0d;
    }

    public interface Shooter {
        double kMaxShooterSpeed = 0.5d;
        double kTurretTolerance = 0.5d;
        double kHoodTolerance = 0.5d;

        public interface TurretPID extends BasePID {}
        public interface HoodPID extends BasePID {}

    }
}   