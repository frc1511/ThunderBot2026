package frc.util.Thunder;

import java.util.function.Supplier;

public class Modifiable {
    private Supplier<Object> m_value;

    public Modifiable(String name, ThunderSubsystem subsystem, Supplier<Object> value) {
        m_value = value;
        subsystem.registerField(name, this);
    }

    public Object getValue() {
        return m_value.get();
    }

    public void withValue(Supplier<Object> value) {
        m_value = value;
    }
}
