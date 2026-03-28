package frc.util.Thunder;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.util.Constants.Status;

public interface ThunderInterface extends Subsystem {
    Status status();
    
    /** High density data logging periodic */
    public void hddlPeriodic();
}
