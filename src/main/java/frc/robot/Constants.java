package frc.robot;

import com.techhounds.houndutil.houndlib.swerve.KrakenCoaxialSwerveModule;
import com.techhounds.houndutil.houndlog.logitems.TunableDouble;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {
    enum ControllerType {
        XboxController,
        FlightStick
    }

    public static final boolean DEBUG_MODE = false;

    public static final ControllerType CONTROLLER_TYPE = ControllerType.FlightStick;

    public static final class Drivetrain {
        public static final int GYRO_DEVICE_ID = 0; // Untested

        public static final boolean FRONT_LEFT_DRIVE_INVERTED = false; // Untested
        public static final boolean FRONT_LEFT_STEER_INVERTED = true; // Untested
        public static final boolean FRONT_LEFT_CANCODER_INVERTED = true; // Untested
        public static final boolean FRONT_RIGHT_DRIVE_INVERTED = false; // Untested
        public static final boolean FRONT_RIGHT_STEER_INVERTED = true; // Untested
        public static final boolean FRONT_RIGHT_CANCODER_INVERTED = true; // Untested
        public static final boolean BACK_LEFT_DRIVE_INVERTED = false; // Untested
        public static final boolean BACK_LEFT_STEER_INVERTED = true; // Untested
        public static final boolean BACK_LEFT_CANCODER_INVERTED = true; // Untested
        public static final boolean BACK_RIGHT_DRIVE_INVERTED = false; // Untested
        public static final boolean BACK_RIGHT_STEER_INVERTED = true; // Untested
        public static final boolean BACK_RIGHT_CANCODER_INVERTED = true; // Untested

        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 0; // Untested
        public static final int FRONT_LEFT_STEER_MOTOR_ID = 0; // Untested
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 0; // Untested
        public static final int FRONT_RIGHT_STEER_MOTOR_ID = 0; // Untested
        public static final int BACK_LEFT_DRIVE_MOTOR_ID = 0; // Untested
        public static final int BACK_LEFT_STEER_MOTOR_ID = 0; // Untested
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 0; // Untested
        public static final int BACK_RIGHT_STEER_MOTOR_ID = 0; // Untested

        public static final int FRONT_LEFT_STEER_ENCODER_ID = 0; // Untested
        public static final int FRONT_RIGHT_STEER_ENCODER_ID = 0; // Untested
        public static final int BACK_LEFT_STEER_ENCODER_ID = 0; // Untested
        public static final int BACK_RIGHT_STEER_ENCODER_ID = 0; // Untested

        public static final double FRONT_LEFT_OFFSET = 0; // Untested
        public static final double FRONT_RIGHT_OFFSET = 0; // Untested
        public static final double BACK_LEFT_OFFSET = 0; // Untested
        public static final double BACK_RIGHT_OFFSET = 0; // Untested

        public static final String CAN_BUS = "canivore";

        /** Distance between left and right wheels. */
        public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(0); // Untested
        /** Distance between front and back wheels. */
        public static final double WHEEL_BASE_METERS = Units.inchesToMeters(0); // Untested

        public static final KrakenCoaxialSwerveModule.SwerveConstants SWERVE_CONSTANTS = new KrakenCoaxialSwerveModule.SwerveConstants();
        static {
            SWERVE_CONSTANTS.DRIVE_kP = 0; // Untested
            SWERVE_CONSTANTS.DRIVE_kI = 0; // Untested
            SWERVE_CONSTANTS.DRIVE_kD = 0; // Untested
            SWERVE_CONSTANTS.DRIVE_kS = 0; // Untested
            SWERVE_CONSTANTS.DRIVE_kV = 0; // Untested
            SWERVE_CONSTANTS.DRIVE_kA = 0; // Untested
            SWERVE_CONSTANTS.STEER_kP = 0; // Untested
            SWERVE_CONSTANTS.STEER_kI = 0; // Untested
            SWERVE_CONSTANTS.STEER_kD = 0; // Untested

            SWERVE_CONSTANTS.DRIVE_GEARING = 0; // Untested
            SWERVE_CONSTANTS.STEER_GEARING = 0; // Untested
            SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * 0; // Untested
            SWERVE_CONSTANTS.DRIVE_ENCODER_ROTATIONS_TO_METERS = SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE
                    / SWERVE_CONSTANTS.DRIVE_GEARING;
            SWERVE_CONSTANTS.STEER_ENCODER_ROTATIONS_TO_RADIANS = 2 * Math.PI / SWERVE_CONSTANTS.STEER_GEARING;

            SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND = 4.282;
            SWERVE_CONSTANTS.MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
            SWERVE_CONSTANTS.MAX_STEER_VELOCITY_RADIANS_PER_SECOND = 8.829 * Math.PI;
            // max velocity in 1/3 sec
            SWERVE_CONSTANTS.MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 8.829 * 3 * Math.PI;

            SWERVE_CONSTANTS.DRIVE_CURRENT_LIMIT = 50;
            SWERVE_CONSTANTS.STEER_CURRENT_LIMIT = 20;
            SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR = DCMotor.getKrakenX60(1);
            SWERVE_CONSTANTS.STEER_GEARBOX_REPR = DCMotor.getKrakenX60(1);
            SWERVE_CONSTANTS.DRIVE_MOI = 0; // Untested
            SWERVE_CONSTANTS.STEER_MOI = 0; // Untested
        }

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = Math.PI / 4; // Untested
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2 * Math.PI;

        public static final Translation2d[] SWERVE_MODULE_LOCATIONS = new Translation2d[] {
                new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
                new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
                new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
                new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2) };

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                SWERVE_MODULE_LOCATIONS[0],
                SWERVE_MODULE_LOCATIONS[1],
                SWERVE_MODULE_LOCATIONS[2],
                SWERVE_MODULE_LOCATIONS[3]);

        public static final double XY_kP = 1;
        public static final double XY_kI = 0;
        public static final double XY_kD = 0;
        public static final TrapezoidProfile.Constraints XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
                SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND,
                SWERVE_CONSTANTS.MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final double THETA_kP = 1;
        public static final double THETA_kI = 0;
        public static final double THETA_kD = 0.1;
        public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                20 * Math.PI, 20 * Math.PI);

        public static final double PATH_FOLLOWING_TRANSLATION_kP = 7;
        public static final double PATH_FOLLOWING_ROTATION_kP = 7;
    }

    public static final class Teleop {
        /**
         * A value inputted into the rate limiter (the joystick input) can move from 0
         * to 1 in 1/RATE_LIMIT seconds.
         * 
         * A rate limit of 3, for example, means that 0->1 in 1/3 sec.
         * Larger numbers mean less of a rate limit.
         */
        public static final double JOYSTICK_INPUT_RATE_LIMIT = 15.0;
        public static final double JOYSTICK_CURVE_EXP = 2;
        public static final TunableDouble INPUT_LIMIT = new TunableDouble("Drivetrain", "Input Limit", 1);
    }
}
