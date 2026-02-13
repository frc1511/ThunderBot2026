package frc.util;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.util.Constants.Status;

public interface ThunderSubsystem extends Subsystem {
    Status status();
}
