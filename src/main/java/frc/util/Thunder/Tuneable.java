package frc.util.Thunder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.util.Alert;
import frc.util.Helpers;

public class Tuneable<T extends Object> {
    static public ArrayList<Tuneable<? extends Object>> tuneables = new ArrayList<Tuneable<? extends Object>>();

    public T value;
    public String m_name;
    public boolean m_hasUpdated = true;

    public Tuneable(T value_, String name) {
        if (!allowedTypes.contains(value_.getClass().getName())) throw new IllegalArgumentException("Invalid type for this tunable");
        value = value_;
        m_name = name;
        tuneables.add(this);
    }

    public T get() {
        return value;
    }

    public void set(T value_) {
        value = value_;
    }

    public void update() {
        update_(safer(this));
    }

    static List<String> allowedTypes = Arrays.asList(
        Double.class.getName(),  Double[].class.getName(),
        Boolean.class.getName(), Boolean[].class.getName(),
        String.class.getName(),  String[].class.getName(),
        Byte[].class.getName());

    static private void update_(Tuneable<Object> t) {
        String key = "Tunables / " + t.m_name;
        NetworkTableEntry e = SmartDashboard.getEntry(key);
        boolean ntOutdated = t.m_hasUpdated;
        Object currentValue = t.value;
        String className = currentValue.getClass().getName();


        if (className == allowedTypes.get(0)) {        // Double
            if (ntOutdated) {t.m_hasUpdated = false; e.setNumber((double)currentValue);}
            else t.value = e.getNumber((double)currentValue);

        } else if (className == allowedTypes.get(1)) { // Double[]
            if (ntOutdated) {t.m_hasUpdated = false; e.setDoubleArray((double[])currentValue);}
            else t.value = e.getDoubleArray((double[])currentValue);

        } else if (className == allowedTypes.get(2)) { // Boolean
            if (ntOutdated) {t.m_hasUpdated = false; e.setBoolean((boolean)currentValue);}
            else t.value = e.getBoolean((boolean)currentValue);

        } else if (className == allowedTypes.get(3)) { // Boolean[]
            if (ntOutdated) {t.m_hasUpdated = false; e.setBooleanArray((boolean[])currentValue);}
            else t.value = e.getBooleanArray((boolean[])currentValue);
            
        } else if (className == allowedTypes.get(4)) { // String
            if (ntOutdated) {t.m_hasUpdated = false; e.setString((String)currentValue);}
            else t.value = e.getString((String)currentValue);
            
        } else if (className == allowedTypes.get(5)) { // String[]
            if (ntOutdated) {t.m_hasUpdated = false; e.setStringArray((String[])currentValue);}
            else t.value = e.getStringArray((String[])currentValue);
            
        } else if (className == allowedTypes.get(6)) { // Byte[]
            if (ntOutdated) {t.m_hasUpdated = false; e.setRaw((byte[])currentValue);}
            else t.value = e.getRaw((byte[])currentValue);
            
        } else {
            Alert.info("Exausted tunables types, you might have put in an invalid type");
        }
    }

    static public void periodic() {
        if (Helpers.isPitModeEnabled()) {
            tuneables.forEach(t -> {
                Tuneable.update_(safer(t));
            });
        }
    }

    static private Tuneable<Object> safer(Tuneable<?> t) {
        @SuppressWarnings("unchecked")
        Tuneable<Object> safe = (Tuneable<Object>)t;
        return safe;
    }
}
