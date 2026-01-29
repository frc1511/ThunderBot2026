package frc.robot.subsytem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlexExternalEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

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

    public Moving() {
        m_motor = new SparkMax(1, MotorType.kBrushless);
        switchLow = new DigitalInput(3);
        Trigger switchLowTrueFalse = new Trigger(switchLow::get);
        switchHigh = new DigitalInput(2);                               // Triggers seem to allow for onTrue to be called but il ifnish it later - oliver
        Trigger switchHighTrueFalse = new Trigger(switchHigh::get);
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
            .isFinished(() -> !highFlipped());
    }
    public Command runBackward() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_motor.set(-0.5);
            })
            .isFinished(() -> !lowFlipped());
    }
    public Command stopMotor() {
        return new CommandBuilder(this)
            .onExecute(() -> {
                m_motor.stopMotor();
            })
            .isFinished(() -> true);
    }
}