package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.util.CommandBuilder;
import frc.util.Thunder.Command;
import frc.util.Thunder.ThunderSubsystem;

public class ManualModeHandler {
    private Command m_subsystem_1_manual;
    private Command m_subsystem_2_manual;

    private double m_subsystem_1_velocity = 0.1;
    private double m_subsystem_2_velocity = 0.1;

    private double m_subsystem_1_speed_increment = 0.1;
    private double m_subsystem_2_speed_increment = 0.1;

    private double m_direction_mult = 1;

    private ArrayList<ThunderSubsystem> m_subsystems;

    private SendableChooser<Command> m_chooser_1;
    private SendableChooser<Command> m_chooser_2;
    
    public ManualModeHandler(ArrayList<ThunderSubsystem> subsystems) {
        m_subsystems = subsystems;
        assert m_subsystems.size() >= 2;
        m_subsystem_1_manual = m_subsystems.get(0).manual(() -> 0);
        m_subsystem_2_manual = m_subsystems.get(1).manual(() -> 0);

        m_chooser_1 = new SendableChooser<Command>();
        m_chooser_2 = new SendableChooser<Command>();

        m_subsystems.forEach((subsystem) -> {
            m_chooser_1.addOption(subsystem.getName(), subsystem.manual(() ->  m_direction_mult * m_subsystem_1_velocity));
            m_chooser_2.addOption(subsystem.getName(), subsystem.manual(() ->  m_direction_mult * m_subsystem_2_velocity));
        });

        m_chooser_1.onChange((subsystem) -> {
            m_subsystem_1_manual = subsystem;
        });

        m_chooser_2.onChange((subsystem) -> {
            m_subsystem_2_manual = subsystem;
        });

        SmartDashboard.putData("Manual / Subsytem 1", m_chooser_1);
        SmartDashboard.putData("Manual / Subsytem 2", m_chooser_2);
    }

    public void periodic() {
        SmartDashboard.putNumber("Manual / Subsystem 1 Speed", m_subsystem_1_velocity);
        SmartDashboard.putNumber("Manual / Subsystem 2 Speed", m_subsystem_2_velocity);

        SmartDashboard.putString("Manual / Subsystem 1 Actual Name", m_chooser_1.getSelected().getName());
        SmartDashboard.putString("Manual / Subsystem 2 Actual Name", m_chooser_2.getSelected().getName());
    }

    public Command increaseManual1Speed() {
        return new InstantCommand(() -> 
            m_subsystem_1_velocity = Math.min(1, m_subsystem_1_velocity + m_subsystem_1_speed_increment));
    }

    public Command decrementManual1Speed() {
        return new InstantCommand(() -> 
            m_subsystem_1_velocity = Math.max(0, m_subsystem_1_velocity - m_subsystem_1_speed_increment));
    }

    public Command increaseManual2Speed() {
        return new InstantCommand(() -> 
            m_subsystem_2_velocity = Math.min(1, m_subsystem_2_velocity + m_subsystem_2_speed_increment));
    }

    public Command decrementManual2Speed() {
        return new InstantCommand(() -> 
            m_subsystem_2_velocity = Math.max(0, m_subsystem_2_velocity - m_subsystem_2_speed_increment));
    }

    ///////

    public Command runSubsystem1Forward() {
        m_direction_mult = 1;
        return m_subsystem_1_manual;
    }

    public Command runSubsystem1Reverse() {
        m_direction_mult = -1;
        return m_subsystem_1_manual.manual(() -> -m_subsystem_1_velocity);
    }

    public Command runSubsystem2Forward() {
        return m_subsystem_2.manual(() ->  m_subsystem_2_velocity);
    }

    public Command runSubsystem2Reverse() {
        return m_subsystem_2.manual(() -> -m_subsystem_2_velocity);
    }

    //////
    
    public Command manual1ModifierLow() {
        return new CommandBuilder().onExecute(() -> m_subsystem_1_speed_increment = 0.01).onEnd(() -> m_subsystem_1_speed_increment = 0.1);
    }

    public Command manual1ModifierSuperLow() {
        return new CommandBuilder().onExecute(() -> m_subsystem_1_speed_increment = 0.001).onEnd(() -> m_subsystem_1_speed_increment = 0.1);
    }

    public Command manual2ModifierLow() {
        return new CommandBuilder().onExecute(() -> m_subsystem_2_speed_increment = 0.01).onEnd(() -> m_subsystem_2_speed_increment = 0.1);
    }
    
    public Command manual2ModifierSuperLow() {
        return new CommandBuilder().onExecute(() -> m_subsystem_2_speed_increment = 0.001).onEnd(() -> m_subsystem_2_speed_increment = 0.1);
    }
}
