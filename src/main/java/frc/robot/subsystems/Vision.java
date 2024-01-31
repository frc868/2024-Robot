package frc.robot.subsystems;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera;
import com.techhounds.houndutil.houndlib.subsystems.BaseVision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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

    public Pose3d[] getCameraPoses() {
    }

    public Pose3d[] getAprilTagPoses() {
    }

    public void setPoseEstimator(SwerveDrivePoseEstimator poseEstimator) {
    }

    public void setSimPoseSupplier(Supplier<Pose2d> simPoseSupplier) {
    }
}
