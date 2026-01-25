package frc.util;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.util.Elastic.Notification;
import frc.util.Elastic.NotificationLevel;

public class Alert {

    private static HashMap<Integer, Double> notifications = new HashMap<Integer, Double>();

    public static void info(String desc) {
        auto(NotificationLevel.INFO, desc);
    }

    public static void warning(String desc) {
        auto(NotificationLevel.WARNING, desc);
    }

    public static void error(String desc) {
        auto(NotificationLevel.ERROR, desc);
    }

    public static void auto(NotificationLevel notificationLevel, String desc) {
        StackTraceElement[] stackTrace = Thread.currentThread().getStackTrace();

        String loc = "Unknown";
        if (stackTrace.length >= 2) {
            String testFile = stackTrace[2].getFileName();
            if (testFile.contains("Alert") && stackTrace.length >= 3) {
                loc = String.format("In %s at line %s", stackTrace[3].getFileName().replace(".java", ""), stackTrace[3].getLineNumber());
            } else {
                loc = String.format("In %s at line %s", stackTrace[2].getFileName().replace(".java", ""), stackTrace[2].getLineNumber());
            }
        }

        full(notificationLevel, loc, desc);
    }

    public static void full(NotificationLevel notificationLevel, String title, String desc) {
        int displayTime = 2000;
        if (notificationLevel == NotificationLevel.WARNING) displayTime = 3000;
        if (notificationLevel == NotificationLevel.ERROR) displayTime = 5000;
        sendNotification(notificationLevel, title, desc, displayTime);
    }

    private static void sendNotification(NotificationLevel notificationLevel, String title, String desc, int displayTimeMillis) {
        int hash = title.hashCode() + desc.hashCode() + notificationLevel.hashCode() + displayTimeMillis;

        if (notifications.containsKey(hash)) {
            double lastNotificationSendTime = notifications.get(hash).doubleValue();
            if (Timer.getFPGATimestamp() - lastNotificationSendTime < Constants.kAntiSpamAlertTimeout) {
                return; // No spam plz :(
            }
        }
        notifications.put(hash, Double.valueOf(Timer.getFPGATimestamp()));

        if (notificationLevel == NotificationLevel.INFO) {
            System.out.println(String.format("[INFO] %s: %s", title, desc));
        } else if (notificationLevel == NotificationLevel.WARNING) {
            DriverStation.reportWarning(String.format("[WARN] %s: %s", title, desc), false);
        } else {
            DriverStation.reportError(String.format("[ERROR] %s: %s", title, desc), false);
        }

        Elastic.sendNotification(new Notification(notificationLevel, title, desc, displayTimeMillis));
    }
}
