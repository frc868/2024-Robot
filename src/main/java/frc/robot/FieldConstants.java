package frc.robot;

import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

@LoggedObject
public class FieldConstants {
    public static final double FIELD_LENGTH = Units.inchesToMeters(651.223);
    public static final double FIELD_WIDTH = Units.inchesToMeters(323.277);

    @Log
    public static final Pose3d TOP_LEFT_OPENING = new Pose3d(
            Units.inchesToMeters(18.11),
            Units.inchesToMeters(197.73),
            Units.inchesToMeters(82.90),
            new Rotation3d());
    @Log
    public static final Pose3d TOP_RIGHT_OPENING = new Pose3d(
            Units.inchesToMeters(18.11),
            Units.inchesToMeters(239.11),
            Units.inchesToMeters(82.90), new Rotation3d());
    @Log
    public static final Pose3d BOTTOM_LEFT_OPENING = new Pose3d(
            0,
            Units.inchesToMeters(197.73),
            Units.inchesToMeters(78.13),
            new Rotation3d());
    @Log
    public static final Pose3d BOTTOM_RIGHT_OPENING = new Pose3d(
            0,
            Units.inchesToMeters(239.11),
            Units.inchesToMeters(78.13),
            new Rotation3d());
    @Log
    public static final Pose3d TARGET = new Pose3d(
            Units.inchesToMeters(6),
            Units.inchesToMeters(218.42),
            Units.inchesToMeters(78.13 + 1),
            new Rotation3d());
}
