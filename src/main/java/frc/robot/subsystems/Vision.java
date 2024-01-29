package frc.robot.subsystems;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlib.subsystems.BaseVision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class Vision implements BaseVision {
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
