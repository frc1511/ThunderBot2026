package frc.util;

public class Broken {
    public static void autoFullDisable() {
        autoShooterFullDisable();
        autoHangFullDisable();
    }

    /** Disable everything drive related */
    public static boolean drivetrainFullyDisabled = false;

    /** Disable everything shooter related */
    public static boolean shooterFullyDisabled = false;
    /** Disable shooter motor A */
    public static boolean shooterADisabled = false;
    /** Disable shooter motor B */
    public static boolean shooterBDisabled = false;

    public static void autoShooterFullDisable() {
        shooterFullyDisabled = shooterADisabled && shooterBDisabled; // Fully disable if both motors are disabled
    }

    /** Disable everything hood related */
    public static boolean hoodDisabled = true;

    public static final boolean turretDisable = true;

    /** Disable everything intake related */
    public static boolean intakeDisabled = true;
    /** Disable just the pivot for intake */
    public static boolean pivotDisabled = true;

    /** Disable everything hang related */
    public static boolean hangFullyDisabled = true;
    /** Disable the upper hang mag limit */
    public static boolean hangUpperLimitDisabled = true;
    /** Disable the lower hang mag limit */
    public static boolean hangLowerLimitDisabled = true;

    public static void autoHangFullDisable() {
        hangFullyDisabled = hangUpperLimitDisabled && hangLowerLimitDisabled; // Fully disable if both of the sensors are dead b/c atp its proabably a bit too unsafe
    }

    /** Disable everything kicker related */
    public static boolean kickerDisabled = true;

    /** Disable everything spindexer related */
    public static boolean spindexerDisabled = true;
}
