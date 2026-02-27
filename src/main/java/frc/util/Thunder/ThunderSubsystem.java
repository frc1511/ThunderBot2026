package frc.util.Thunder;

import java.util.HashMap;

import frc.util.Constants.Status;

public class ThunderSubsystem implements ThunderInterface {
    private HashMap<String, Modifiable<Object>> m_fields;

    public void registerField(String key, Modifiable<Object> field) {
        m_fields.put(key, field);
    }

    public Modifiable<Object> getField(String key) {
        if (m_fields.containsKey(key)) {
            return m_fields.get(key);
        }
        return null;
    }

    public Status status() {
        throw new UnsupportedOperationException("ThunderSubsystems must define status()");
    }
}
