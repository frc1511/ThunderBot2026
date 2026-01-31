package frc.robot.orchestration.Autonomous;

import com.thunder.lib.auto.ThunderAutoProject;

import frc.robot.orchestration.Conductor;
import frc.util.Alert;

public class AutoLoader {
    private AutoLoader() {} // No instantiation for you :3

    public static ThunderAutoProject load(Conductor conductor) {
        ThunderAutoProject autoProject = new ThunderAutoProject("paths");

        if (autoProject.isLoaded()) {
            autoProject.registerActionCommand("Shoot", conductor.hub.shoot());

            
        } else {
            Alert.warning("Auto failed to load");
        }

        return autoProject;
    }
}
