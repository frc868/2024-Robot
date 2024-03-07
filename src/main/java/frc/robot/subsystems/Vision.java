package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;

import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera;
import com.techhounds.houndutil.houndlib.TriConsumer;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Vision.*;

@LoggedObject
public class Vision extends SubsystemBase {
    private SwerveDrivePoseEstimator poseEstimator = null;
    private TriConsumer<Pose2d, Double, Matrix<N3, N1>> visionMeasurementConsumer = null;
    private Supplier<Pose2d> simPoseSupplier = null;
    private final VisionSystemSim visionSim = new VisionSystemSim("main");

    @Log(groups = "cameras")
    private final AprilTagPhotonCamera houndeye01 = new AprilTagPhotonCamera("HoundEye01",
            ROBOT_TO_CAMS[0], CAMERA_CONSTANTS, 0.64, 0.22);
    @Log(groups = "cameras")
    private final AprilTagPhotonCamera houndeye02 = new AprilTagPhotonCamera("HoundEye02",
            ROBOT_TO_CAMS[1], CAMERA_CONSTANTS, 0.64, 0.22);
    @Log(groups = "cameras")
    private final AprilTagPhotonCamera houndeye03 = new AprilTagPhotonCamera("HoundEye03",
            ROBOT_TO_CAMS[2], CAMERA_CONSTANTS, 0.64, 0.22);

    private final AprilTagPhotonCamera[] cameras = new AprilTagPhotonCamera[] {
            houndeye01, houndeye02, houndeye03 };

    public Vision() {
        AprilTagFieldLayout tagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        // "x": -0.038099999999999995,
        // "y": 5.547867999999999,
        // "z": 1.4511020000000001
        // AprilTagFieldLayout tagLayout = new AprilTagFieldLayout(
        // List.of(
        // new AprilTag(7,
        // new Pose3d(0, 5.547867999999999-4.982717999999999,
        // 1.4511020000000001,
        // new Rotation3d(new Quaternion(1, 0, 0, 0)))),
        // new AprilTag(8,
        // new Pose3d(0, ,
        // 1.4511020000000001,
        // new Rotation3d(new Quaternion(1, 0, 0, 0))))),
        // 16.541, 8.211);
        if (RobotBase.isSimulation()) {
            visionSim.addAprilTags(tagLayout);
            for (AprilTagPhotonCamera camera : cameras) {
                visionSim.addCamera(camera.getSim(), camera.getRobotToCam());
            }
        }
    }

    @Override
    public void periodic() {
        updatePoseEstimator();
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(simPoseSupplier.get());
    }

    public void updatePoseEstimator() {
        if (poseEstimator == null) {
            return;
        }

        Pose2d prevEstimatedRobotPose = poseEstimator.getEstimatedPosition();
        for (AprilTagPhotonCamera photonCamera : cameras) {
            Optional<EstimatedRobotPose> result = photonCamera
                    .getEstimatedGlobalPose(prevEstimatedRobotPose);

            if (result.isPresent()) {
                EstimatedRobotPose estPose = result.get();
                Pose2d oldPose = estPose.estimatedPose.toPose2d();
                Pose2d pose = new Pose2d(oldPose.getX() + Units.inchesToMeters(1.5), oldPose.getY(),
                        oldPose.getRotation());

                Matrix<N3, N1> stddevs = photonCamera.getEstimationStdDevs(pose,
                        SINGLE_TAG_STD_DEVS, MULTI_TAG_STD_DEVS);
                visionMeasurementConsumer.accept(pose, estPose.timestampSeconds, stddevs);
            }
        }
    }

    @Log
    public Pose3d[] cameraPoses() {
        List<Pose3d> poses = new ArrayList<Pose3d>();
        for (Transform3d transform : ROBOT_TO_CAMS) {
            poses.add(new Pose3d(poseEstimator.getEstimatedPosition()).plus(transform)
                    .plus(new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.PI))));
        }
        Pose3d[] poseArray = new Pose3d[poses.size()];
        return poses.toArray(poseArray);
    }

    // @Log
    public Pose3d[] aprilTagPoses() {
        List<Pose3d> poses = new ArrayList<Pose3d>();
        for (AprilTag tag : AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTags()) {
            poses.add(tag.pose);
        }
        Pose3d[] poseArray = new Pose3d[poses.size()];
        return poses.toArray(poseArray);
    }

    public void setPoseEstimator(SwerveDrivePoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;
    }

    public void setVisionMeasurementConsumer(TriConsumer<Pose2d, Double, Matrix<N3, N1>> visionMeasurementConsumer) {
        this.visionMeasurementConsumer = visionMeasurementConsumer;
    }

    public void setSimPoseSupplier(Supplier<Pose2d> simPoseSupplier) {
        this.simPoseSupplier = simPoseSupplier;
    }

    @Log
    public Pose3d[] getEstimatedRobotPoses() {
        return new Pose3d[] {
                houndeye01.getLoggedEstimatedRobotPose(),
                houndeye02.getLoggedEstimatedRobotPose(),
                houndeye03.getLoggedEstimatedRobotPose() };
    }

    @Log
    public Pose3d[] getDetectedAprilTags() {
        return Arrays
                .stream(new Pose3d[][] {
                        houndeye01.getLoggedDetectedAprilTags(),
                        houndeye02.getLoggedDetectedAprilTags(),
                        houndeye03.getLoggedDetectedAprilTags() })
                .flatMap(i -> Arrays.stream(i))
                .toArray(Pose3d[]::new);

    }
}
