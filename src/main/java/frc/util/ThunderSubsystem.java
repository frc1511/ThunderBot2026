package frc.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ThunderSubsystem extends SubsystemBase {
    private boolean m_isDisabled = false;

    public boolean getDisabled() {
        return m_isDisabled;
    }

    public void setDisabled(boolean isDisabled) {
        m_isDisabled = isDisabled;
    }
}
