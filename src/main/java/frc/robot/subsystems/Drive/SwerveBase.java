package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import frc.util.Constants.Swerve;

public class SwerveBase extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>{
    public SwerveBase() {
        super(
            TalonFX::new, TalonFX::new, CANcoder::new,
            Swerve.kDrivetrainConstants, Swerve.FrontLeft, Swerve.FrontRight, Swerve.BackLeft, Swerve.BackRight
        );
    }
}
