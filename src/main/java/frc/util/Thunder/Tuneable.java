package frc.util.Thunder;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Tuneable<T> {
    static public ArrayList<Tuneable> tuneables = new ArrayList<Tuneable>();

    public T m_value;
    public String m_name;

    public Tuneable(T value, String name) {
        m_value = value;
        m_name = name;
        tuneables.add(this);
    }

    public T get() {
        return m_value;
    }

    public void set(T value) {
        m_value = value;
    }

    // static public periodic() {
    //     for (Tuneable t : tuneables) {
    //         SmartDashboard.get
    //     }
    // } 
    // TODO: Remind me if I don't get back to this please
}
