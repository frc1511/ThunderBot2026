import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.HashSet;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.util.Constants;
import frc.util.FiringTable;
import frc.util.FiringTable.FiringDataPoint;

public class TOFConvergenceTest {
    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    }

    private FiringTable firingTable = new FiringTable();

    private Pair<FiringDataPoint, Double> converge(ChassisSpeeds currentSpeed, Translation2d robotPosition, Translation2d targetPosition) {
        ArrayList<Translation2d> iterations = new ArrayList<Translation2d>();

        Translation2d initialDeltaTranslation = targetPosition.minus(robotPosition);

        Translation2d currentDeltaPosition = initialDeltaTranslation;
        Double previousTimeOfFlight = null;

        for (int iteration = 0; iteration < Constants.Swerve.kTimeOfFlightConvergenceMaxRecursions; iteration++) {
            double previousTau;
            if (previousTimeOfFlight != null) {
                previousTau = previousTimeOfFlight.doubleValue();
            } else {
                double deltaDistance = currentDeltaPosition.getNorm();
                previousTau = firingTable.lerp(deltaDistance).timeOfFlight;
            }

            Translation2d virtualTargetOffset = new Translation2d(
                previousTau * -currentSpeed.vxMetersPerSecond,
                previousTau * -currentSpeed.vyMetersPerSecond
            );

            Translation2d virtualTargetPosition = targetPosition.plus(virtualTargetOffset);

            Translation2d virtualTargetDelta = virtualTargetPosition.minus(robotPosition);
            double deltaDistance = virtualTargetDelta.getNorm();
            double newTau = firingTable.lerp(deltaDistance).timeOfFlight;

            Translation2d realTrajectoryEnd = new Translation2d(
                virtualTargetPosition.getX() + currentSpeed.vxMetersPerSecond * newTau,
                virtualTargetPosition.getY() + currentSpeed.vyMetersPerSecond * newTau
            );

            iterations.add(realTrajectoryEnd);

            if (previousTimeOfFlight != null && Math.abs(newTau - previousTau) < Constants.Swerve.kTimeOfFlightConvergenceTolerance) {
                break;
            }
            
            currentDeltaPosition = virtualTargetPosition.minus(robotPosition);

            previousTimeOfFlight = newTau;
        }

        Translation2d finalTargetPosition = iterations.get(iterations.size() - 1);

        Translation2d deltaTarget = finalTargetPosition.minus(robotPosition);
        double finalDistance = deltaTarget.getNorm();
        double finalTheta = deltaTarget.getAngle().getRadians();

        FiringDataPoint finalPoint = firingTable.lerp(finalDistance);
        
        return new Pair<FiringTable.FiringDataPoint,Double>(finalPoint, finalTheta);
    }

    @Test
    void ConvergenceTest() {
        HashSet<Pair<Pose2d, ChassisSpeeds>> testValues = new HashSet<>();
        testValues.add(new Pair<Pose2d,ChassisSpeeds>(
            new Pose2d(Constants.Swerve.blueHubCenterPose.getTranslation().minus(new Translation2d(2, 0)), Rotation2d.kZero),
            new ChassisSpeeds(1, 1, 0)
        ));
        testValues.add(new Pair<Pose2d,ChassisSpeeds>(
            new Pose2d(2, 2, new Rotation2d(0)),
            new ChassisSpeeds(1, 1, 0)
        ));
        testValues.add(new Pair<Pose2d,ChassisSpeeds>( // 8800 rpm
            new Pose2d(1, 3, new Rotation2d(0)),
            new ChassisSpeeds(-1, 1, 0)
        ));
        testValues.add(new Pair<Pose2d,ChassisSpeeds>( // 1500 deg hood + 150000000 rpm
            new Pose2d(4, 1, new Rotation2d(0)),
            new ChassisSpeeds(0, 3, 0)
        ));
        testValues.add(new Pair<Pose2d,ChassisSpeeds>( // Insanely high
            new Pose2d(2, 1, new Rotation2d(0)),
            new ChassisSpeeds(-5, -3, 0)
        ));

        testValues.forEach(pair -> {
            Pose2d pose = pair.getFirst();
            ChassisSpeeds speeds = pair.getSecond();
            Pose2d hub = Constants.Swerve.blueHubCenterPose;

            Pair<FiringDataPoint, Double> convergeResult = converge(speeds, pose.getTranslation(), hub.getTranslation());

            System.out.println("Speed: " + convergeResult.getFirst().speedRPM);
            System.out.println("Hood: " + convergeResult.getFirst().hoodAngle);
            System.out.println("Rbt Angle: " + convergeResult.getSecond());

            assertTrue(convergeResult.getFirst().speedRPM < 5000);
            assertTrue(convergeResult.getFirst().speedRPM > 0);
            assertTrue(convergeResult.getFirst().hoodAngle < Constants.Hood.Position.TOP.get());
            assertTrue(convergeResult.getFirst().hoodAngle > Constants.Hood.Position.BOTTOM.get());
        });
    }
}
