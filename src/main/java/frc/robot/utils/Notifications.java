package frc.robot.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.utils.libraries.Elastic;
import frc.robot.utils.libraries.Elastic.Notification;
import frc.robot.utils.libraries.Elastic.Notification.NotificationLevel;
import java.util.ArrayList;
import java.util.List;

public class Notifications {
    private static List<Alert> alerts = new ArrayList<>();

    public static void send(NotificationLevel type, String title, String description) {
        send(type, title, description, 0); // Yes 0 does work it makes it infinite
    }

    public static void send(NotificationLevel level, String title, String description, int desplayTimeMillis) {
        Notification notification = new Notification(level, title, description, desplayTimeMillis);
        Elastic.sendNotification(notification);

        AlertType alertType;
        switch (level) {
            case INFO:
                alertType = AlertType.kInfo;
                break;
            case WARNING:
                alertType = AlertType.kWarning;
                break;
            case ERROR:
                alertType = AlertType.kError;
                break;
            default:
                alertType = AlertType.kInfo;
                break;
        }

        Alert alert = new Alert(title + " - " + description, alertType);
        alerts.add(alert);
        alert.set(true);
    }
}
