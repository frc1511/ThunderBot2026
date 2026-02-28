package frc.robot.orchestration.Autonomous;

import com.thunder.lib.auto.ThunderAutoProject;

import frc.robot.Robot;
import frc.util.Alert;

public class AutoLoader {
    private AutoLoader() {} // No instantiation for you :3

    public static ThunderAutoProject load(Robot robot) {
        ThunderAutoProject autoProject = new ThunderAutoProject("paths");

        if (autoProject.isLoaded()) {
            autoProject.registerActionCommand("Shoot", robot.firingOrchestrator.fire());
            autoProject.registerActionCommand("HangUp", robot.hang.extend());
            autoProject.registerActionCommand("HangDown", robot.hang.retract());
            autoProject.registerActionCommand("Intake", robot.hungerOrchestrator.consume());
            autoProject.registerActionCommand("StopIntake", robot.hungerOrchestrator.excuseYourself());
        } else {
            Alert.warning("Auto failed to load");
        }

        return autoProject;
    }
}
