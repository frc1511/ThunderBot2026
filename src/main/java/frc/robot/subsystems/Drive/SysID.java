package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SysID {
    private final SwerveSubsystem m_Subsystem;

    // Swerve requests to apply during SysId characterization
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private SysIdRoutine m_sysIdRoutineTranslation;
    private SysIdRoutine m_sysIdRoutineSteer;
    private SysIdRoutine m_sysIdRoutineRotation;

    public SysID(SwerveSubsystem subsystem) {
        m_Subsystem = subsystem;

        // SysId routine for characterizing translation. This is used to find PID gains for the drive motors.
        m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(4),
                null,        // Use default timeout (10 s)
                state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                output -> m_Subsystem.setControl(m_translationCharacterization.withVolts(output)),
                null,
                m_Subsystem
            )
        );

        // SysId routine for characterizing steer. This is used to find PID gains for the steer motors.
        m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(7),
                null,        // Use default timeout (10 s)
                state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                volts -> m_Subsystem.setControl(m_steerCharacterization.withVolts(volts)),
                null,
                m_Subsystem
            )
        );

        /*
        * SysId routine for characterizing rotation.
        * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
        * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
        */
        m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(Math.PI / 6).per(Second),
                Volts.of(Math.PI),
                null, // Use default timeout (10 s)
                state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                output -> {
                    m_Subsystem.setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                    SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                },
                null,
                m_Subsystem
            )
        );

    }

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineSteer;

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }
}
