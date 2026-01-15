package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import frc.util.Constants.SwerveConstants;

public class SwerveBase extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>{
    public SwerveBase() {
        super(
            TalonFX::new, TalonFX::new, CANcoder::new,
            SwerveConstants.kDrivetrainConstants, SwerveConstants.FrontLeft, SwerveConstants.FrontRight, SwerveConstants.BackLeft, SwerveConstants.BackRight
        );
    }
}
