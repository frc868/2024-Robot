package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;

import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Vision.*;

@LoggedObject
public class Vision extends SubsystemBase {
    private SwerveDrivePoseEstimator poseEstimator = null;
    private Supplier<Pose2d> simPoseSupplier = null;
    private final VisionSystemSim visionSim = new VisionSystemSim("main");

    @Log(name = "houndeye01", groups = "cameras")
    private final AprilTagPhotonCamera photonCamera1 = new AprilTagPhotonCamera("HoundEye01",
            ROBOT_TO_CAMS[0], CAMERA_CONSTANTS, 0.64, 0.22);
    @Log(name = "houndeye02", groups = "cameras")
    private final AprilTagPhotonCamera photonCamera2 = new AprilTagPhotonCamera("HoundEye02",
            ROBOT_TO_CAMS[1], CAMERA_CONSTANTS, 0.64, 0.22);
    @Log(name = "houndeye03", groups = "cameras")
    private final AprilTagPhotonCamera photonCamera3 = new AprilTagPhotonCamera("HoundEye03",
            ROBOT_TO_CAMS[2], CAMERA_CONSTANTS, 0.64, 0.22);

    private final AprilTagPhotonCamera[] photonCameras = new AprilTagPhotonCamera[] {
            photonCamera1, photonCamera2, photonCamera3 };

    public Vision() {
        AprilTagFieldLayout tagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        visionSim.addAprilTags(tagLayout);
        for (AprilTagPhotonCamera photonCamera : photonCameras) {
            visionSim.addCamera(photonCamera.getSim(), photonCamera.getRobotToCam());
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
        for (AprilTagPhotonCamera photonCamera : photonCameras) {
            Optional<EstimatedRobotPose> result = photonCamera
                    .getEstimatedGlobalPose(prevEstimatedRobotPose);

            if (result.isPresent()) {
                EstimatedRobotPose estPose = result.get();
                Matrix<N3, N1> stddevs = photonCamera.getEstimationStdDevs(estPose.estimatedPose.toPose2d(),
                        SINGLE_TAG_STD_DEVS, MULTI_TAG_STD_DEVS);
                poseEstimator.addVisionMeasurement(estPose.estimatedPose.toPose2d(),
                        estPose.timestampSeconds, stddevs);
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

    @Log
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

    public void setSimPoseSupplier(Supplier<Pose2d> simPoseSupplier) {
        this.simPoseSupplier = simPoseSupplier;
    }
}
