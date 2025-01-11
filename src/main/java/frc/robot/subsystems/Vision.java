package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;

import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera;
import com.techhounds.houndutil.houndlib.TriConsumer;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Vision.*;

@LoggedObject
public class Vision extends SubsystemBase {
    /**
     * The pose estimator to receive the latest robot position from (used for
     * logging and the "closest to last pose" strategy).
     */
    private SwerveDrivePoseEstimator poseEstimator = null;
    /**
     * The consumer for vision measurements, taking in the pose, the timestamp, and
     * the standard deviations.
     */
    private TriConsumer<Pose2d, Double, Matrix<N3, N1>> visionMeasurementConsumer = null;
    /**
     * Supplier for drivetrain chassis speeds, used to disable pose estimation when
     * moving too quickly.
     */
    private Supplier<ChassisSpeeds> chassisSpeedsSupplier = null;
    /**
     * Pose supplier for a ground source of truth pose via odometry, for simulation.
     */
    private Supplier<Pose2d> simPoseSupplier = null;
    /** A simulation of the vision system. */
    private final VisionSystemSim visionSim = new VisionSystemSim("main");

    @Log(groups = "cameras")
    private final AprilTagPhotonCamera houndeye01 = new AprilTagPhotonCamera("HoundEye01",
            ROBOT_TO_CAMS[0], CAMERA_CONSTANTS, 0.2, 0.1);
    @Log(groups = "cameras")
    private final AprilTagPhotonCamera houndeye02 = new AprilTagPhotonCamera("HoundEye02",
            ROBOT_TO_CAMS[1], CAMERA_CONSTANTS, 0.2, 0.1);
    @Log(groups = "cameras")
    private final AprilTagPhotonCamera houndeye03 = new AprilTagPhotonCamera("HoundEye03",
            ROBOT_TO_CAMS[2], CAMERA_CONSTANTS, 0.2, 0.1);

    private final AprilTagPhotonCamera[] cameras = new AprilTagPhotonCamera[] {
            houndeye01, houndeye02, houndeye03 };

    public Vision() {
        AprilTagFieldLayout tagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        if (RobotBase.isSimulation()) {
            visionSim.addAprilTags(tagLayout);
            for (AprilTagPhotonCamera camera : cameras) {
                visionSim.addCamera(camera.getSim(), camera.getRobotToCam());
            }
        }
    }

    /**
     * Updates vision estimates periodically.
     */
    @Override
    public void periodic() {
        updateVisionEstimates();
    }

    /**
     * Updates the vision simulation with the latest ground truth pose.
     */
    @Override
    public void simulationPeriodic() {
        visionSim.update(simPoseSupplier.get());
    }

    /**
     * Updates the measurement consumer with the latest data from all cameras.
     */
    public void updateVisionEstimates() {
        if (poseEstimator == null) {
            return;
        }

        Pose2d prevEstimatedRobotPose = poseEstimator.getEstimatedPosition();
        for (AprilTagPhotonCamera photonCamera : cameras) {
            Optional<EstimatedRobotPose> result = photonCamera
                    .getEstimatedGlobalPose(prevEstimatedRobotPose);

            if (result.isPresent()) {
                EstimatedRobotPose estPose = result.get();
                Pose2d pose = estPose.estimatedPose.toPose2d();

                Matrix<N3, N1> stddevs = photonCamera.getEstimationStdDevs(pose,
                        SINGLE_TAG_STD_DEVS,
                        DriverStation.isAutonomous() ? MULTI_TAG_STD_DEVS : MULTI_TAG_TELEOP_STD_DEVS);

                double normSpeed = new Translation2d(chassisSpeedsSupplier.get().vxMetersPerSecond,
                        chassisSpeedsSupplier.get().vyMetersPerSecond).getNorm();
                if (normSpeed < 0.5 || !DriverStation.isAutonomous()) {
                    if (photonCamera.getName() == "HoundEye01" || !DriverStation.isAutonomous()) {
                        visionMeasurementConsumer.accept(pose, Timer.getFPGATimestamp(), stddevs);
                    }
                }
            }
        }
    }

    /**
     * Gets the supplied camera poses in the global frame, based off of the robot
     * pose.
     * 
     * @return the poses of each registered camera
     */
    public Pose3d[] cameraPoses() {
        List<Pose3d> poses = new ArrayList<Pose3d>();
        for (Transform3d transform : ROBOT_TO_CAMS) {
            poses.add(new Pose3d(poseEstimator.getEstimatedPosition()).plus(transform)
                    .plus(new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.PI))));
        }
        Pose3d[] poseArray = new Pose3d[poses.size()];
        return poses.toArray(poseArray);
    }

    /**
     * Gets the poses of all AprilTags in the current field layout.
     * 
     * @return the poses of all AprilTags
     */
    public Pose3d[] aprilTagPoses() {
        List<Pose3d> poses = new ArrayList<Pose3d>();
        for (AprilTag tag : AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTags()) {
            poses.add(tag.pose);
        }
        Pose3d[] poseArray = new Pose3d[poses.size()];
        return poses.toArray(poseArray);
    }

    /**
     * Sets the pose estimator to use for the vision system.
     * 
     * @param poseEstimator the pose estimator to use
     */
    public void setPoseEstimator(SwerveDrivePoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;
    }

    /**
     * Sets the consumer for vision measurements, taking in the pose, the timestamp,
     * and the standard deviations of a given measurement.
     * 
     * @param visionMeasurementConsumer the consumer to use
     */
    public void setVisionMeasurementConsumer(TriConsumer<Pose2d, Double, Matrix<N3, N1>> visionMeasurementConsumer) {
        this.visionMeasurementConsumer = visionMeasurementConsumer;
    }

    /**
     * Sets the supplier for the ground truth simulation pose.
     * 
     * @param simPoseSupplier the pose supplier to use
     */
    public void setSimPoseSupplier(Supplier<Pose2d> simPoseSupplier) {
        this.simPoseSupplier = simPoseSupplier;
    }

    /**
     * Sets the supplier for the robot's chassis speeds, used to invalidate pose
     * measurements when the robot is moving too fast.
     * 
     * @param chassisSpeedsSupplier the supplier to use
     */
    public void setChassisSpeedsSupplier(Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    }

    /**
     * Gets an aggregated list of the latest cached measurements from all cameras,
     * so that they can be displayed easily. Does not actually update the cameras.
     * 
     * @return the latest cached measurements from all cameras
     */
    @Log
    public Pose3d[] getEstimatedRobotPoses() {
        return new Pose3d[] {
                houndeye01.getLoggedEstimatedRobotPose(),
                houndeye02.getLoggedEstimatedRobotPose(),
                houndeye03.getLoggedEstimatedRobotPose() };
    }

    /**
     * Gets an aggregated list of all of the detected AprilTags from the latest
     * cached measurements from all cameras, so that they can be displayed easily.
     * Does not actually update the cameras.
     * 
     * @return the latest cached detected AprilTags from all cameras
     */
    @Log
    public Pose3d[] getDetectedAprilTags() {
        Pose3d[][] detectedTags = {
                houndeye01.getLoggedDetectedAprilTags(),
                houndeye02.getLoggedDetectedAprilTags(),
                houndeye03.getLoggedDetectedAprilTags()
        };

        int totalSize = 0;
        for (Pose3d[] tags : detectedTags) {
            totalSize += tags.length;
        }

        Pose3d[] result = new Pose3d[totalSize];

        int index = 0;
        for (Pose3d[] tags : detectedTags) {
            for (Pose3d tag : tags) {
                result[index++] = tag;
            }
        }

        return result;
    }
}
