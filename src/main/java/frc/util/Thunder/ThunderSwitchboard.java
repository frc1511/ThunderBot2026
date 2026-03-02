package frc.util.Thunder;

import java.util.HashMap;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ThunderSwitchboard implements Sendable {
    private final CommandGenericHID m_hid;
    private HashMap<Integer, ThunderSwitch> m_switches = new HashMap<Integer, ThunderSwitch>();

    public ThunderSwitchboard(int port) {
        m_hid = new CommandGenericHID(port);
    }

    /**
     * With the returned {@code ThunderSwitch}, use the {@code .getOn()} method to get an always up to date value of the switch
     */
    public ThunderSwitch button(int slot) {
        ThunderSwitch button = new ThunderSwitch(m_hid.button(slot));
        m_switches.put(slot, button);
        return button;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ThunderSwitchboard");

        m_switches.forEach((slot, button) -> {
            builder.addBooleanProperty(String.format("Slot %d", slot), button::isOn, null);
        });
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
            m_trigger.onChange(new InstantCommand(() -> {m_value = trigger.getAsBoolean();}).ignoringDisable(true));
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
