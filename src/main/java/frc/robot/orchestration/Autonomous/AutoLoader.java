package frc.robot.orchestration.Autonomous;

import com.thunder.lib.auto.ThunderAutoProject;

import frc.robot.Robot;
import frc.util.Alert;

public class AutoLoader {
    private AutoLoader() {} // No instantiation for you :3

    public static ThunderAutoProject load(Robot robot) {
        ThunderAutoProject autoProject = new ThunderAutoProject("paths");

        if (autoProject.isLoaded()) {
            autoProject.registerActionCommand("Shoot", robot.cannonOrchestrator.shootTurret());
        } else {
            Alert.warning("Auto failed to load");
        }

        return autoProject;
    }
}
