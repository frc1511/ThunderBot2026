package frc.robot.orchestration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Cannon.HoodSubsystem;
import frc.robot.subsystems.Cannon.ShooterSubsystem;
import frc.robot.subsystems.Cannon.TurretSubsystem;
import frc.util.CommandBuilder;

public class CannonOrchestrator {
    private HoodSubsystem hood;
    private ShooterSubsystem shooter;
    private TurretSubsystem turret;

    public CannonOrchestrator(Robot robot) {
        hood = robot.hood;
        shooter = robot.shooter;
        turret = robot.turret;
    }

    public static class Orientation {
        public double turretDeg;
        public double hoodDeg;

        public Orientation(double turretDeg_, double hoodDeg_) {
            turretDeg = turretDeg_;
            hoodDeg = hoodDeg_;
        }
    }

    public Command moveToOrientation(Orientation orientation) {
        return hood.toPosition(() -> orientation.hoodDeg)
            .alongWith(turret.toPosition(() -> orientation.turretDeg));                
    }

    public Command shootTurret() {
        return new CommandBuilder()
            .onExecute(() -> {
                if (shooter.shooterAtSpeed() && hood.hoodAtPosition() && turret.turretAtPosition()) {
                    shooter.shoot();
                }
            })
            .isFinished(true);
    }
}
