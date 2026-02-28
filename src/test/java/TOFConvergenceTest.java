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

    private Pair<FiringDataPoint, Double> converge(ChassisSpeeds currentSpeed, Translation2d robotPosition, Translation2d targetPosition, int recursions) {
        // These are relative velocities of the hub to the robot. The hub is not moving, but from the robot's perspective it is moving with the opposite velocity of the robot.
        double relativeVelocityOfHubX = -currentSpeed.vxMetersPerSecond;
        double relativeVelocityOfHubY = -currentSpeed.vyMetersPerSecond;

        double dX = robotPosition.getX() - targetPosition.getX();
        double dY = robotPosition.getY() - targetPosition.getY();

        FiringDataPoint interpolatedDataPoint = firingTable.lerp(targetPosition.getDistance(robotPosition));

        double timeOfFlight = interpolatedDataPoint.timeOfFlight;
    
        double dXPredicted = dX + relativeVelocityOfHubX * timeOfFlight;
        double dYPredicted = dY + relativeVelocityOfHubY * timeOfFlight;
    
        double distPredicted = Math.sqrt(Math.pow(dYPredicted, 2) + Math.pow(dXPredicted, 2));
    
        FiringDataPoint nextInterpolatedDataPoint = firingTable.lerp(distPredicted);

        double nextTimeOfFlight = nextInterpolatedDataPoint.timeOfFlight;

        double timeOfFlightChange = nextTimeOfFlight - timeOfFlight;

        if (Math.abs(timeOfFlightChange) > Constants.Swerve.kTimeOfFlightConvergenceTolerance && !(recursions >= Constants.Swerve.kTimeOfFlightConvergenceMaxRecursions)) {
            Translation2d newTargetPosition = new Translation2d(dXPredicted, dYPredicted);
            return converge(currentSpeed, robotPosition, newTargetPosition, recursions + 1);
        } else {
            return new Pair<FiringDataPoint, Double>(nextInterpolatedDataPoint, Math.atan(dYPredicted / dXPredicted));
        }
    }

    @Test
    void ConvergenceTest() {
        HashSet<Pair<Pose2d, ChassisSpeeds>> testValues = new HashSet<>();
        testValues.add(new Pair<Pose2d,ChassisSpeeds>(
            new Pose2d(2, 2, new Rotation2d(0)),
            new ChassisSpeeds(1, 1, 0)
        ));
        testValues.add(new Pair<Pose2d,ChassisSpeeds>(
            new Pose2d(1, 3, new Rotation2d(0)),
            new ChassisSpeeds(-1, 1, 0)
        ));
        testValues.add(new Pair<Pose2d,ChassisSpeeds>(
            new Pose2d(4, 1, new Rotation2d(0)),
            new ChassisSpeeds(0, 3, 0)
        ));

        testValues.forEach(pair -> {
            Pose2d pose = pair.getFirst();
            ChassisSpeeds speeds = pair.getSecond();
            Pose2d hub = Constants.Swerve.blueHubCenterPose;

            double returnedSpeed = converge(speeds, pose.getTranslation(), hub.getTranslation(), 0).getFirst().speedRPM;

            System.out.println(returnedSpeed);
            System.out.println(converge(speeds, pose.getTranslation(), hub.getTranslation(), 0).getSecond());
        });
    }
}
