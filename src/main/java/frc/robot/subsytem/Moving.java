package frc.robot.subsytem;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlexExternalEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.util.CommandBuilder;
import frc.util.Constants.IOMap;

public class Moving extends SubsystemBase {
    public SparkMax m_motor;
    public DigitalInput switchLow;
    public DigitalInput switchHigh;
    public Trigger switchLowTrueFalse, switchHighTrueFalse;

    public Moving() {
        m_motor = new SparkMax(1, MotorType.kBrushless);
        
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kCoast);
        m_motor.configure(cfg, com.revrobotics.ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);  
        switchLow = new DigitalInput(1);
        switchLowTrueFalse = new Trigger(switchLow::get);
        switchHigh = new DigitalInput(2);                               // Triggers seem to allow for onTrue to be called but il ifnish it later - oliver
        switchHighTrueFalse = new Trigger(switchHigh::get);
    }

    public boolean lowFlipped() {
        return !switchLow.get();
    }

    public boolean highFlipped() {
        return !switchHigh.get();
    }

    public Command runForward() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_motor.set(0.5);
            })
            .isFinished(() -> true);
    }
    public Command runBackward() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_motor.set(-0.5);
            })
            .isFinished(() -> true);
    }
    public Command stopMotor() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_motor.stopMotor();
            })
            .isFinished(() -> true);
    }
}