package frc.robot;

import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera.PhotonCameraConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class Vision {
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

        public static final PhotonCameraConstants CAMERA_CONSTANTS = new PhotonCameraConstants();
        static {
            CAMERA_CONSTANTS.WIDTH = 1600;
            CAMERA_CONSTANTS.HEIGHT = 1200;
            CAMERA_CONSTANTS.FOV = 95.39;
            CAMERA_CONSTANTS.FPS = 30;
            CAMERA_CONSTANTS.AVG_LATENCY = 30;
            CAMERA_CONSTANTS.STDDEV_LATENCY = 15;
        }

        // NOT FINAL! PLEASE CHANGE
        // TODO

        /** Calculations for transforming from camera location to robot location */
        public static final Transform3d[] ROBOT_TO_CAMS = new Transform3d[] {
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(9.5), Units.inchesToMeters(9.5),
                                Units.inchesToMeters(10)),
                        new Rotation3d(0, -Units.degreesToRadians(10), -Math.PI / 8)),
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(-12.5), Units.inchesToMeters(0),
                                Units.inchesToMeters(10)),
                        new Rotation3d(0, -Units.degreesToRadians(10), Math.PI)),
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(9.5), Units.inchesToMeters(-9.5),
                                Units.inchesToMeters(10)),
                        new Rotation3d(0, -Units.degreesToRadians(10), Math.PI / 8))
        };

    }

}
