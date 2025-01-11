package frc.robot;

import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/**
 * Contains constants that map out the 2024 field and its elements.
 */
@LoggedObject
public class FieldConstants {
    public static final double FIELD_LENGTH = Units.inchesToMeters(651.223);
    public static final double FIELD_WIDTH = Units.inchesToMeters(323.277);

    @Log
    public static final Pose3d TOP_LEFT_SPEAKER_OPENING = new Pose3d(
            Units.inchesToMeters(18.11),
            Units.inchesToMeters(197.73),
            Units.inchesToMeters(82.90),
            new Rotation3d());
    @Log
    public static final Pose3d TOP_RIGHT_SPEAKER_OPENING = new Pose3d(
            Units.inchesToMeters(18.11),
            Units.inchesToMeters(239.11),
            Units.inchesToMeters(82.90), new Rotation3d());
    @Log
    public static final Pose3d BOTTOM_LEFT_SPEAKER_OPENING = new Pose3d(
            0,
            Units.inchesToMeters(197.73),
            Units.inchesToMeters(78.13),
            new Rotation3d());
    @Log
    public static final Pose3d BOTTOM_RIGHT_SPEAKER_OPENING = new Pose3d(
            0,
            Units.inchesToMeters(239.11),
            Units.inchesToMeters(78.13),
            new Rotation3d());
    @Log
    public static final Pose3d SPEAKER_TARGET = new Pose3d(
            Units.inchesToMeters(6),
            Units.inchesToMeters(218.42),
            Units.inchesToMeters(78.13 + 1),
            new Rotation3d());

    @Log
    public static final Pose2d STAGE_CENTER = new Pose2d(
            Units.inchesToMeters(191.640),
            Units.inchesToMeters(161.645),
            new Rotation2d());
    @Log
    public static final Pose2d FAR_CHAIN_CENTER = new Pose2d(
            Units.inchesToMeters(230.20),
            Units.inchesToMeters(161.61),
            Rotation2d.fromDegrees(0));
    @Log
    public static final Pose2d LEFT_CHAIN_CENTER = new Pose2d(
            Units.inchesToMeters(172.37),
            Units.inchesToMeters(195.044),
            Rotation2d.fromDegrees(120));
    @Log
    public static final Pose2d RIGHT_CHAIN_CENTER = new Pose2d(
            Units.inchesToMeters(172.37),
            Units.inchesToMeters(128.246),
            Rotation2d.fromDegrees(-120));
}
