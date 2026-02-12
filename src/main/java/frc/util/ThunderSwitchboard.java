package frc.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ThunderSwitchboard {
    private final CommandGenericHID m_hid;

    public ThunderSwitchboard(int port) {
        m_hid = new CommandGenericHID(port);
    }

    /**
     * With the returned {@code ThunderSwitch}, use the {@code .getOn()} method to get an always up to date value of the switch
     */
    public ThunderSwitch button(int slot) {
        return new ThunderSwitch(m_hid.button(slot));
    }

    /**
     * ThunderSwitch lets you have a variable that is always up to date with the state of a {@code Trigger}
     */
    public class ThunderSwitch {
        private boolean m_value;
        private Trigger m_trigger;

        public ThunderSwitch(Trigger trigger) {
            m_trigger = trigger;

            m_value = m_trigger.getAsBoolean();
            m_trigger.onChange(new InstantCommand(() -> {m_value = trigger.getAsBoolean();}));
        }

        public Trigger get() {
            return m_trigger;
        }

        public boolean isOn() {
            return m_value;
        }

        public boolean isOff() {
            return !m_value;
        }
    }
}
