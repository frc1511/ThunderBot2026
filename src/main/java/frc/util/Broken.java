package frc.util;

public class Broken {
    /** Disable everything drive related */
    public static boolean drivetrainFullyDisabled = true;

    /** Disable everything shooter related */
    public static boolean shooterFullyDisabled = false; // NOTE: YOU MUST BE CAREFUL NOT TO HAVE THIS FALSE WHEN A & B ARE TRUE
    /** Disable shooter motor A */
    public static boolean shooterADisabled = false;
    /** Disable shooter motor B */
    public static boolean shooterBDisabled = false;

    /** Disable everything hood related */
    public static boolean hoodDisabled = false;
    public static boolean hoodBeamBreakDisabled = false;

    public static final boolean turretDisable = true;

    /** Disable everything intake related */
    public static boolean intakeDisabled = false;
    /** Disable just the pivot for intake */
    public static boolean pivotDisabled = false;

    /** Disable everything hang related */
    public static boolean hangFullyDisabled = false;
    /** Disable the upper hang mag limit */
    public static boolean hangUpperLimitDisabled = false;
    /** Disable the lower hang mag limit */
    public static boolean hangLowerLimitDisabled = false;

    /** Disable everything kicker related */
    public static boolean kickerDisabled = false;

    /** Disable everything spindexer related */
    public static boolean spindexerDisabled = false;
}
