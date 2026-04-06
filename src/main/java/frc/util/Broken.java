package frc.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Broken {
    /** Disable everything drive related */
    public static boolean drivetrainFullyDisabled = false;

    /** Disable everything aux related */
    public static boolean auxFullyDisabled = false;

    /** Disable everything shooter related */
    public static boolean shooterFullyDisabled = false; //! NOTE: YOU MUST BE CAREFUL NOT TO HAVE THIS FALSE WHEN A & B ARE TRUE
    /** Disable shooter motor A */
    public static boolean shooterADisabled = false;
    /** Disable shooter motor B */
    public static boolean shooterBDisabled = false;

    /** Disable everything hood related */
    public static boolean hoodDisabled = false;
    public static boolean hoodBeamBreakDisabled = false;

    public static final boolean turretDisable = true;

    /** Disable everything intake related */
    public static boolean intakeFullyDisabled = false; //! NOTE: YOU MUST BE CAREFUL NOT TO HAVE THIS FALSE WHEN LEFT & RIGHT ARE TRUE
    /** Disable left intake motor */
    public static boolean intakeLeftDisabled = false;
    /** Disable left intake motor */
    public static boolean intakeRightDisabled = false;
    /** Disable just the pivot for intake */
    public static boolean pivotDisabled = false;

    /** Disable everything hang related */
    public static boolean hangFullyDisabled = true;
    /** Disable the upper hang mag limit */
    public static boolean hangUpperLimitDisabled = false;
    /** Disable the lower hang mag limit */
    public static boolean hangLowerLimitDisabled = false;

    /** Disable everything kicker related */
    public static boolean kickerDisabled = false;

    /** Disable everything spindexer related */
    public static boolean spindexerDisabled = false;

    /** Disable everything LED related */
    public static boolean blinkyBlinkyDisabled = false;
    public static boolean blinkyBlinkyButtonBopped = false;

    public static boolean blinkyBlinkyDisableStatus() {
        if(blinkyBlinkyDisabled || blinkyBlinkyButtonBopped) {
            return true;
        } else {
            return false;
        }
    }

    public static Sendable getBrokenStatuses() {
        return new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addBooleanProperty("Drivetrain Fully Disabled", () -> drivetrainFullyDisabled, null);
                builder.addBooleanProperty("Aux Fully Disabled", () -> auxFullyDisabled, null);
                builder.addBooleanProperty("Shooter Fully Disabled", () -> shooterFullyDisabled, null);
                builder.addBooleanProperty("Shooter motor A Disabled", () -> shooterADisabled, null);
                builder.addBooleanProperty("Shooter motor B Disabled", () -> shooterBDisabled, null);
                builder.addBooleanProperty("Hood Disabled", () -> hoodDisabled, null);
                builder.addBooleanProperty("Hood Beam Break Disabled", () -> hoodBeamBreakDisabled, null);
                builder.addBooleanProperty("Turret Disabled", () -> turretDisable, null);
                builder.addBooleanProperty("Intake Fully Disabled", () -> intakeFullyDisabled, null);
                builder.addBooleanProperty("Intake Left Motor Disabled", () -> intakeLeftDisabled, null);
                builder.addBooleanProperty("Intake Right Motor Disabled", () -> intakeRightDisabled, null);
                builder.addBooleanProperty("Pivot Disabled", () -> pivotDisabled, null);
                builder.addBooleanProperty("Hang Fully Disabled", () -> hangFullyDisabled, null);
                builder.addBooleanProperty("Hang Upper Limit Disabled", () -> hangUpperLimitDisabled, null);
                builder.addBooleanProperty("Hang Lower Limit Disabled", () -> hangLowerLimitDisabled, null);
                builder.addBooleanProperty("Kicker Disabled", () -> kickerDisabled, null);
                builder.addBooleanProperty("Spindexer Disabled", () -> spindexerDisabled, null);
                builder.addBooleanProperty("Blinky Blinky Disabled", () -> blinkyBlinkyDisabled, null);
                builder.addBooleanProperty("Blinky Blinky Lite™ Disabled", () -> blinkyBlinkyDisableStatus(), null);
            }
        };
    }
}
