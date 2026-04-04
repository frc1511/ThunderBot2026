package frc.util.Thunder;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.util.Constants.Status;

public class ThunderSubsystem implements ThunderInterface {
    private HashMap<String, Modifiable> m_fields = new HashMap<>();

    private boolean m_hasBeenRegistered = false;

    public void registerSubsystem() {
        if (!m_hasBeenRegistered) {
            CommandScheduler.getInstance().registerSubsystem(this);
            m_hasBeenRegistered = true;
        }
    }

    public boolean isRegistered() {
        return m_hasBeenRegistered;
    }

    public void registerField(String key, Modifiable field) {
        m_fields.put(key, field);
    }

    public Modifiable getField(String key) {
        if (m_fields.containsKey(key)) {
            return m_fields.get(key);
        }
        return null;
    }

    public Status status() {
        throw new UnsupportedOperationException("ThunderSubsystems must define status()");
    }

    /** High density data logging periodic */
    public void hddlPeriodic() {

    }
}
