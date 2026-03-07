package frc.robot.orchestration.Autonomous;

import com.thunder.lib.auto.ThunderAutoProject;

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
            autoProject.registerActionCommand("Shoot", robot.firingOrchestrator.fireThenStop());
            autoProject.registerActionCommand("HangUp", robot.hang.extend());
            autoProject.registerActionCommand("HangDown", robot.hang.retract());
            autoProject.registerActionCommand("Intake", robot.intake.eatStart());
            autoProject.registerActionCommand("StopIntake", robot.intake.stopEating());
            autoProject.registerActionCommand("Wait_short", new WaitCommand(5));
            autoProject.registerActionCommand("Stop_shoot", robot.firingOrchestrator.halt());
            autoProject.registerActionCommand("Pivot_down", robot.pivot.down());
            autoProject.registerActionCommand("Pivot_up", robot.pivot.up());
            autoProject.registerActionCommand("Pivot_down_soft", robot.pivot.downSoft());
            autoProject.registerActionCommand("Pivot_up_soft", robot.pivot.upSoft());
        } else {
            Alert.warning("Auto failed to load");
        }

        return autoProject;
    }
}
