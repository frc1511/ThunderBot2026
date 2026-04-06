package frc.robot.subsystems.Hang;

import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.util.Broken;
import frc.util.CommandBuilder;
import frc.util.Constants;
import frc.util.Helpers;
import frc.util.Constants.HangConstants;
import frc.util.Constants.IOMap;
import frc.util.Constants.Status;
import frc.util.Constants.HangConstants.HangPID;
import frc.util.Thunder.ThunderSubsystem;

public class HangSubsystem extends ThunderSubsystem {
    private SparkMax m_motor;
    private RelativeEncoder m_encoder;
    private SparkClosedLoopController m_pidController;

    private DigitalInput m_lowerLimitSensor;
    private DigitalInput m_upperLimitSensor;
    private AnalogInput m_distanceSensor;
    private boolean m_isZeroed;
    private boolean m_isClimbing;

    public HangSubsystem() {
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true);
        motorConfig.closedLoop
            .pid(HangPID.kP, HangPID.kI, HangPID.kD)
            .allowedClosedLoopError(Constants.HangConstants.kSetpointPositionTolerance, ClosedLoopSlot.kSlot0)
            .outputRange(Constants.HangConstants.kMaxPullSpeed, Constants.HangConstants.kMaxDeploySpeed);
        
        m_isZeroed = false;

        if (!Broken.hangFullyDisabled) {
            m_motor = new SparkMax(IOMap.Hang.kHangMotor, MotorType.kBrushless);
            m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            m_encoder = m_motor.getEncoder();
            m_pidController = m_motor.getClosedLoopController();

            // For broken functionality, these still "work" when the sensors are disconnected so we don't have to null them to avoid errors
            m_lowerLimitSensor = new DigitalInput(IOMap.Hang.kDIOlowerLimit);
            m_upperLimitSensor = new DigitalInput(IOMap.Hang.kDIOupperLimit);
            m_distanceSensor = new AnalogInput(IOMap.Hang.kAnalogDistance);
        }

        if (Broken.hangLowerLimitDisabled || Helpers.isBypassModeEnabled()) {
            m_isZeroed = true;
        }
    }

    @Override
    public void periodic() {
        if (Broken.hangFullyDisabled) return;

        SmartDashboard.putBoolean("Hang / At Lower Sensor", isAtLowerLimit());
        SmartDashboard.putBoolean("Hang / At Upper Sensor", isAtUpperLimit());
        SmartDashboard.putBoolean("Hang / Is Zeroed", isZeroed());
        SmartDashboard.putNumber("Hang / Position", m_encoder.getPosition());
        SmartDashboard.putNumber("Hang / Output %", m_motor.getAppliedOutput());
        SmartDashboard.putNumber("Hang / Pid Setpoint", m_pidController.getSetpoint());
        SmartDashboard.putBoolean("Hang / At Setpoint", atSetpoint());
        SmartDashboard.putNumber("Hang / Putput A", m_motor.getOutputCurrent());
        SmartDashboard.putNumber("Hang / Distance Sensor Distance", getDistanceSensor().getFirst());
        SmartDashboard.putBoolean("Hang / Distance Sensor Valid", getDistanceSensor().getSecond());
    }

    private boolean isAtLowerLimit() {
        if (Broken.hangLowerLimitDisabled) return false;
        if (Broken.hangFullyDisabled) return true;

        return !m_lowerLimitSensor.get();
    }

    private boolean isAtUpperLimit() {
        if (Broken.hangUpperLimitDisabled) return false;
        if (Broken.hangFullyDisabled) return true;

        return !m_upperLimitSensor.get();
    }

    public boolean isZeroed() {
        return m_isZeroed;
    }

    public Command zeroHang() {
        if (Broken.hangFullyDisabled || Broken.hangLowerLimitDisabled || Helpers.isBypassModeEnabled()) {
            m_isZeroed = true;
            return CommandBuilder.none(this);
        }

        return new CommandBuilder(this)
            .onExecute(() -> {
                if (Helpers.isBypassModeEnabled()) {
                    stop();
                    return;
                }
                if (isAtLowerLimit()) {
                    stop();
                    return;
                }
                m_isClimbing = true;

                m_motor.set(HangConstants.kZeroingSpeed);
            })
            .isFinished(() -> isAtLowerLimit() || Helpers.isBypassModeEnabled())
            .onEnd((boolean interupped) -> {
                stop();
                if (!interupped) {
                    m_encoder.setPosition(0);
                    m_isZeroed = true;
                }
            })
            .onlyIf(() -> !m_isZeroed);
    }

    private boolean atSetpoint() {
        if (Broken.hangFullyDisabled) return true;

        return m_pidController.isAtSetpoint() && Math.abs(m_motor.getAppliedOutput()) < Constants.HangConstants.kSetpointMaxVelocity;
    }

    private boolean atExtensionLimit() {
        if (Broken.hangFullyDisabled) return true;

        return (atSetpoint() && m_pidController.getSetpoint() == HangConstants.kMaxDeployDistanceRotations) || (isAtUpperLimit() && !Helpers.isBypassModeEnabled());
    }

    public Command extend() { // Hang will die if it goes past the upper limit
        if (Broken.hangFullyDisabled) return CommandBuilder.none(this);
        
        return new CommandBuilder(this)
            .onExecute(() -> {
                if (atExtensionLimit()) {
                    stop();
                    return;
                }
                m_isClimbing = true;

                m_pidController.setSetpoint(HangConstants.kMaxDeployDistanceRotations, ControlType.kPosition);
            })
            .isFinished(this::atExtensionLimit)
            .onlyIf(this::isZeroed);
    }

    public boolean hasFinishedExtending() {
        return atExtensionLimit();
    }

    private boolean atRetractionLimit() {
        if (Broken.hangFullyDisabled) return true;

        return (atSetpoint() && m_pidController.getSetpoint() == HangConstants.kMaxPullDistanceRotations) || (isAtLowerLimit() && !Helpers.isBypassModeEnabled());
    }

    public Command retract() {
        if (Broken.hangFullyDisabled) return CommandBuilder.none(this);

        return new CommandBuilder(this)
            .onExecute(() -> {
                m_isClimbing = true;

                m_pidController.setSetpoint(HangConstants.kMaxPullDistanceRotations, ControlType.kPosition);
                if (atRetractionLimit()) {
                    stop();
                    return;
                }
            })
            .isFinished(this::atRetractionLimit)
            .onlyIf(this::isZeroed);
    }

    
    private double m_beforePosition = 0;

    public Command stowForTrench() {
        if (Broken.hangFullyDisabled) return CommandBuilder.none(this);

        return new CommandBuilder(this)
            .onInitialize(() -> {
                m_beforePosition = m_pidController.getSetpoint();
            })
            .onExecute(() -> {
                if (isAtLowerLimit()) {
                    stop();
                    return;
                }
                m_isClimbing = true;

                m_pidController.setSetpoint(HangConstants.kTrenchSafeDistanceRotations, ControlType.kPosition);
            })
            .onEnd(() -> {
                m_pidController.setSetpoint(m_beforePosition, ControlType.kPosition);
            })
            .onlyIf(this::isZeroed);
    }

    public Command halt() {
        if (Broken.hangFullyDisabled) return CommandBuilder.none(this);

        return new CommandBuilder(this)
            .onExecute(this::stop)
            .isFinished(true);
    }

    private void stop() {
        m_motor.stopMotor();
        m_isClimbing = false;
    }

    public boolean climbClimbingButHasntClumbJustYet() {
        return m_isClimbing;
    }

    public Command manual(DoubleSupplier speedSupplier) {
        if (Broken.hangFullyDisabled) return CommandBuilder.none(this);

        return new CommandBuilder(this)
            .onExecute(() -> {
                double speed = -speedSupplier.getAsDouble() * .5;
                if ((isAtLowerLimit() && speed < 0) || (isAtUpperLimit() && speed > 0) || Helpers.isBypassModeEnabled())
                    stop();
                else {
                    m_isClimbing = true;
                    m_motor.set(speed);
                }
            })
            .isFinished(true)
            .repeatedly();
    }

    public Status status() {
        if (Broken.hangFullyDisabled) return Status.DISABLED;
        if (!Helpers.onCANChain(m_motor)) return Status.DISCONNECTED;
        if (Helpers.isRunning(m_motor)) return Status.ACTIVE;
        return Status.IDLE;
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    private double m_preJostlePosition;
    private double m_i = 0;
    public Command jostle() {
        if (Broken.hangFullyDisabled) return CommandBuilder.none(this);

        return new CommandBuilder(this)
            .onInitialize(() -> {
                m_i = 0;
                m_preJostlePosition = m_pidController.getSetpoint();
            })
            .onExecute(() -> {
                double delta = Constants.HangConstants.kJostleAmplitude * Math.sin(m_i * (Math.PI / 2d));
                m_i += 0.01;
                m_pidController.setSetpoint(m_preJostlePosition + delta, ControlType.kPosition);
            })
            .onEnd(() -> {
                m_pidController.setSetpoint(m_preJostlePosition, ControlType.kPosition);
            })
            .isFinished(true);
    }

    /**
     * In meters
     */
    private LinkedHashSet<Double> m_trackedDistances = new LinkedHashSet<Double>();
    public Pair<Double, Boolean> getDistanceSensor() {
        double volts = m_distanceSensor.getVoltage();

        double centimeters = Math.pow(volts / 9.6654, -0.902);

        centimeters = Helpers.clamp(centimeters, 2.5, 40);

        centimeters -= 3; // Make the distance make more sense

        double meters = centimeters / 100;

        boolean valid = false;
        if (m_trackedDistances.size() < 3) {
            m_trackedDistances.add(meters);
        } else if (m_trackedDistances.size() >= 3) {
            Iterator<Double> iter = m_trackedDistances.iterator();
            iter.next();
            iter.remove();
            m_trackedDistances.add(meters);
        }
        double average = m_trackedDistances.stream().mapToDouble(Double::doubleValue).sum() / m_trackedDistances.size();
        valid = Helpers.standardDeviation(m_trackedDistances, .01) && average <= .20;

        return new Pair<Double, Boolean>(average, valid);
    }
}
