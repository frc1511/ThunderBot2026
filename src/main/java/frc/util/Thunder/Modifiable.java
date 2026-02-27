package frc.util.Thunder;

public class Modifiable<T> {
    private T m_value;

    public Modifiable(String name, ThunderSubsystem subsystem, T value) {
        m_value = value;
        subsystem.registerField(name, this);
    }

    public T getValue() {
        return m_value;
    }

    public void setValue(T value) {
        m_value = value;
    }
}
