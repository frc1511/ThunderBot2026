package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.util.CommandBuilder;
import frc.util.Thunder.ThunderSubsystem;

public class ManualModeHandler {
    private ThunderSubsystem m_subsystem_1;
    private ThunderSubsystem m_subsystem_2;

    private double m_subsystem_1_velocity = 0.1;
    private double m_subsystem_2_velocity = 0.1;

    private double m_subsystem_1_speed_increment = 0.1;
    private double m_subsystem_2_speed_increment = 0.1;

    private ArrayList<ThunderSubsystem> m_subsystems;

    private SendableChooser<ThunderSubsystem> m_chooser_1;
    private SendableChooser<ThunderSubsystem> m_chooser_2;
    
    public ManualModeHandler(ArrayList<ThunderSubsystem> subsystems) {
        m_subsystems = subsystems;
        assert m_subsystems.size() >= 2;
        m_subsystem_1 = m_subsystems.get(0);
        m_subsystem_2 = m_subsystems.get(1);

        m_chooser_1 = new SendableChooser<ThunderSubsystem>();
        m_chooser_2 = new SendableChooser<ThunderSubsystem>();

        m_subsystems.forEach((subsystem) -> {
            m_chooser_1.addOption(subsystem.getName(), subsystem);
            m_chooser_2.addOption(subsystem.getName(), subsystem);
        });

        m_chooser_1.setDefaultOption(m_subsystem_1.getName(), m_subsystem_1);
        m_chooser_1.onChange((subsystem) -> {
            m_subsystem_1 = subsystem;
        });

        m_chooser_2.setDefaultOption(m_subsystem_2.getName(), m_subsystem_2);
        m_chooser_2.onChange((subsystem) -> {
            m_subsystem_2 = subsystem;
        });
    }

    public Command increaseManual1Speed() {
        return new CommandBuilder().onExecute(() -> 
            m_subsystem_1_velocity = Math.max(1, m_subsystem_1_velocity + m_subsystem_1_speed_increment));
    }

    public Command decrementManual1Speed() {
        return new CommandBuilder().onExecute(() -> 
            m_subsystem_1_velocity = Math.min(0, m_subsystem_1_velocity - m_subsystem_1_speed_increment));
    }

    public Command increaseManual2Speed() {
        return new CommandBuilder().onExecute(() -> 
            m_subsystem_2_velocity = Math.max(1, m_subsystem_2_velocity + m_subsystem_2_speed_increment));
    }

    public Command decrementManual2Speed() {
        return new CommandBuilder().onExecute(() -> 
            m_subsystem_2_velocity = Math.min(0, m_subsystem_2_velocity - m_subsystem_2_speed_increment));
    }

    ///////

    public Command runSubsystem1Forward() {
        return m_subsystem_1.manual(() ->  m_subsystem_1_velocity);
    }

    public Command runSubsystem1Reverse() {
        return m_subsystem_1.manual(() -> -m_subsystem_1_velocity);
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
