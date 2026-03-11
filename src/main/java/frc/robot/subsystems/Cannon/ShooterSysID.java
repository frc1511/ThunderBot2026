package frc.robot.subsystems.Cannon;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterSysID {
    private SysIdRoutine m_routine;

    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutAngle m_angle = Radians.mutable(0);
    private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

    public ShooterSysID(ShooterSubsystem shooter) {
        m_routine = new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(10),
                null, // Use default timeout (10 s)
                state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                // Tell SysId how to plumb the driving voltage to the motor(s).
                output -> {
                    System.out.println(output.magnitude());
                    shooter.manual_voltage(output::magnitude).execute();
                },
                // Tell SysId how to record a frame of data for each motor on the mechanism being
                // characterized.
                log -> {
                    // Record a frame for the shooter motor.
                    log.motor("shooter-wheel")
                        .voltage(
                            m_appliedVoltage.mut_replace(
                                shooter.getMotorOutput() * RobotController.getBatteryVoltage(), Volts))
                        .angularPosition(m_angle.mut_replace(shooter.getMotorRotations(), Rotations))
                        .angularVelocity(
                            m_velocity.mut_replace(shooter.getMotorRate(), RotationsPerSecond));
                },
                shooter
            )
        );
    }

    
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_routine.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_routine.dynamic(direction);
    }
}
