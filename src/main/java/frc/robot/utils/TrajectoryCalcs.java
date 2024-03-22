package frc.robot.utils;

import static frc.robot.Constants.Shooter.*;

import com.techhounds.houndutil.houndauto.Reflector;
import com.techhounds.houndutil.houndlib.ChassisAccelerations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;

public class TrajectoryCalcs {

    public static double getTimeToShoot(Pose2d robotPose, Pose3d goalPosition) {
        Transform3d diff = new Pose3d(robotPose).minus(goalPosition);
        double xyDistance = new Translation2d(diff.getX(), diff.getY()).getNorm();
        double distance = diff.getTranslation().getNorm();
        double shooterSpeed = SPEED_INTERPOLATOR.get(xyDistance);
        double noteVelocity = getProjectileSpeed(shooterSpeed);
        double time = distance / noteVelocity;
        return time;
    }

    public static Pose3d calculateEffectiveTargetLocation(Pose2d robotPose, ChassisSpeeds fieldRelRobotVelocity,
            ChassisAccelerations fieldRelRobotAcceleration) {
        Pose3d baseTargetPose = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red
                        ? Reflector.reflectPose3d(FieldConstants.SPEAKER_TARGET,
                                FieldConstants.FIELD_LENGTH)
                        : FieldConstants.SPEAKER_TARGET;

        double shotTime = getTimeToShoot(robotPose, baseTargetPose);

        Pose3d correctedTargetPose = new Pose3d();
        for (int i = 0; i < GOAL_POSITION_ITERATIONS; i++) {
            double virtualGoalX = baseTargetPose.getX()
                    - shotTime * (fieldRelRobotVelocity.vxMetersPerSecond
                            + fieldRelRobotAcceleration.axMetersPerSecondSquared
                                    * ACCELERATION_COMPENSATION_FACTOR);
            double virtualGoalY = baseTargetPose.getY()
                    - shotTime * (fieldRelRobotVelocity.vyMetersPerSecond
                            + fieldRelRobotAcceleration.ayMetersPerSecondSquared
                                    * ACCELERATION_COMPENSATION_FACTOR);

            correctedTargetPose = new Pose3d(virtualGoalX, virtualGoalY, baseTargetPose.getZ(),
                    baseTargetPose.getRotation());

            double newShotTime = getTimeToShoot(robotPose, correctedTargetPose);

            shotTime = newShotTime;
            if (Math.abs(newShotTime - shotTime) <= 0.010) {
                break;
            }
        }

        return correctedTargetPose;
    }
}