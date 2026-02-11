package frc.util;

public class Broken {

    /** Disable everything drive related */
    public static boolean drivetrainFull = false;

    /** Disable everything shooter related */
    public static boolean shooterFull = false;
    /** Disable shooter motor A */
    public static boolean shooterA = false;
    /** Disable shooter motor B */
    public static boolean shooterB = false;

    public static void autoShooterFullDisable() {
        shooterFull = shooterA && shooterB; // Fully disable if both motors are disabled
    }

    /** Disable everything hood related */
    public static boolean hood = true;

    public static final boolean turret = true;

    /** Disable everything intake related */
    public static boolean intakeFull = true;
    /** Disable just the pivot for intake */
    public static boolean pivotFull = true;

    /** Disable everything hang related */
    public static boolean hangFull = true;
    /** Disable the upper hang mag limit */
    public static boolean hangUpper = true;
    /** Disable the lower hang mag limit */
    public static boolean hangLower = true;

    /** Disable everything kicker related */
    public static boolean kicker = true;

    /** Disable everything spindexer related */
    public static boolean spindexer = true;
}
