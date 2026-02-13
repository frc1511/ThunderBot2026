package frc.robot;

import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.ThunderSubsystem;

public class SafetyWatchdog extends SubsystemBase {
    private final Set<ThunderSubsystem> subsystems;
    public SafetyWatchdog(Robot robot) {
        subsystems = Set.of(robot.drivetrain, robot.shooter, robot.hood, robot.turret, robot.spindexer, robot.kicker, robot.intake, robot.pivot, robot.hang);
    }

    @Override
    public void periodic() {
        for (ThunderSubsystem subsystem : subsystems) {
            String subsystemName = subsystem.getName().toLowerCase().replace("subsystem", "");
            SmartDashboard.putString(
                "status_" + subsystemName, 
                subsystem.status().toString());
        }
    }
}
