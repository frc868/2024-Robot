package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera;
import com.techhounds.houndutil.houndlib.subsystems.BaseVision;
import com.techhounds.houndutil.houndlog.interfaces.Log;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase implements BaseVision {
    private AprilTagPhotonCamera camera1 = new AprilTagPhotonCamera(null, null, null, 0, 0);
    private AprilTagPhotonCamera camera2 = new AprilTagPhotonCamera(null, null, null, 0, 0);
    private AprilTagPhotonCamera camera3 = new AprilTagPhotonCamera(null, null, null, 0, 0);

    private SwerveDrivePoseEstimator poseEstimator;

    public Vision() {
    }

    public void updatePoseEstimator() {
    }

    @Log
    public Pose3d[] getCameraPoses() {
    }

    @Log
    public Pose3d[] getAprilTagPoses() {
        List<Pose3d> poses = new ArrayList<Pose3d>();
        for (AprilTag tag : AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTags()) {
            poses.add(tag.pose);
        }
        Pose3d[] poseArray = new Pose3d[poses.size()];
        return poses.toArray(poseArray);
    }

    public void setPoseEstimator(SwerveDrivePoseEstimator poseEstimator) {
    }

    public void setSimPoseSupplier(Supplier<Pose2d> simPoseSupplier) {
    }
}
