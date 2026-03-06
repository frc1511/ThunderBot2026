package frc.robot.orchestration.Autonomous;

import com.thunder.lib.auto.ThunderAutoProject;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.util.Alert;

public class AutoLoader {
    private AutoLoader() {} // No instantiation for you :3

    public static ThunderAutoProject load(Robot robot) {
        ThunderAutoProject autoProject = new ThunderAutoProject("paths");

        if (autoProject.isLoaded()) {
            autoProject.registerActionCommand("DB_Hub_AL_Start", robot.drivetrain.setHubLock(true));
            autoProject.registerActionCommand("DB_Hub_AL_End", robot.drivetrain.setHubLock(false));
            autoProject.registerActionCommand("Shoot", robot.firingOrchestrator.fire());
            autoProject.registerActionCommand("HangUp", robot.hang.extend());
            autoProject.registerActionCommand("HangDown", robot.hang.retract());
            autoProject.registerActionCommand("Intake", robot.hungerOrchestrator.consume());
            autoProject.registerActionCommand("StopIntake", robot.hungerOrchestrator.excuseYourself());
            autoProject.registerActionCommand("Wait_short", new WaitCommand(5));
        } else {
            Alert.warning("Auto failed to load");
        }

        return autoProject;
    }
}
