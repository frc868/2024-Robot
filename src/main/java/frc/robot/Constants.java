package frc.robot;

import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera.PhotonCameraConstants;
import com.techhounds.houndutil.houndlib.leds.BaseLEDSection;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import com.techhounds.houndutil.houndlib.swerve.CoaxialSwerveModule.SwerveConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;

public class Constants {
    enum ControllerType {
        XboxController,
        FlightStick
    }

    public static final boolean DEBUG_MODE = false;

    public static final ControllerType CONTROLLER_TYPE = ControllerType.FlightStick;

    public static final double PERIOD = 0.020;

    public static final class Drivetrain {
        public static enum MusicTrack {
            IMPERIAL_MARCH("imperial_march.chrp"),
            MEGALOVANIA("megalovania.chrp"),
            NATIONAL_ANTHEM("national_anthem.chrp"),
            SHAKE_IT_OFF("shake_it_off.chrp");

            private MusicTrack(String filename) {
                this.filename = filename;
            }

            public final String filename;
        }

        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
        public static final int FRONT_LEFT_STEER_MOTOR_ID = 2;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
        public static final int FRONT_RIGHT_STEER_MOTOR_ID = 4;
        public static final int BACK_LEFT_DRIVE_MOTOR_ID = 5;
        public static final int BACK_LEFT_STEER_MOTOR_ID = 6;
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 7;
        public static final int BACK_RIGHT_STEER_MOTOR_ID = 8;

        public static final int FRONT_LEFT_STEER_ENCODER_ID = 0;
        public static final int FRONT_RIGHT_STEER_ENCODER_ID = 1;
        public static final int BACK_LEFT_STEER_ENCODER_ID = 2;
        public static final int BACK_RIGHT_STEER_ENCODER_ID = 3;

        public static final String CAN_BUS_NAME = "canivore";

        public static final int PIGEON_ID = 0;

        public static final boolean DRIVE_MOTORS_INVERTED = false;
        public static final boolean STEER_MOTORS_INVERTED = true;
        public static final boolean STEER_CANCODERS_INVERTED = RobotBase.isReal() ? false : true;

        // 2/17/24
        // public static final double FRONT_LEFT_OFFSET = 0.457763671875;
        // public static final double FRONT_RIGHT_OFFSET = -0.183349609375;
        // public static final double BACK_LEFT_OFFSET = 0.24267578125;
        // public static final double BACK_RIGHT_OFFSET = 0.48583984375;
        public static final double FRONT_LEFT_OFFSET = 0.4521484375;
        public static final double FRONT_RIGHT_OFFSET = -0.179443359375;
        public static final double BACK_LEFT_OFFSET = 0.242919921875;
        public static final double BACK_RIGHT_OFFSET = 0.498046875;

        /** Distance between left and right wheels. */
        public static final double TRACK_WIDTH_METERS = 0.527;
        /** Distance between front and back wheels. */
        public static final double WHEEL_BASE_METERS = 0.527;
        public static final double DRIVE_BASE_RADIUS_METERS = 0.3727;

        public static final SwerveConstants SWERVE_CONSTANTS = new SwerveConstants();
        static {
            // 2/24/24
            SWERVE_CONSTANTS.DRIVE_kP = 0.84056;
            SWERVE_CONSTANTS.DRIVE_kI = 0.0;
            SWERVE_CONSTANTS.DRIVE_kD = 0.0;
            SWERVE_CONSTANTS.DRIVE_kS = 0.24857;
            SWERVE_CONSTANTS.DRIVE_kV = 0.66681;
            SWERVE_CONSTANTS.DRIVE_kA = 0.10325;
            SWERVE_CONSTANTS.STEER_kP = 100.0;
            SWERVE_CONSTANTS.STEER_kI = 0.0;
            SWERVE_CONSTANTS.STEER_kD = 1.0;
            SWERVE_CONSTANTS.STEER_kS = 0; // TODO
            SWERVE_CONSTANTS.STEER_kV = 0; // TODO
            SWERVE_CONSTANTS.STEER_kA = 0; // TODO

            SWERVE_CONSTANTS.DRIVE_GEARING = 5.357;
            SWERVE_CONSTANTS.STEER_GEARING = 150.0 / 7.0;
            SWERVE_CONSTANTS.COUPLING_RATIO = 50.0 / 16.0;
            SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * 0.04762663828;
            SWERVE_CONSTANTS.DRIVE_ENCODER_ROTATIONS_TO_METERS = SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE
                    / SWERVE_CONSTANTS.DRIVE_GEARING;
            SWERVE_CONSTANTS.STEER_ENCODER_ROTATIONS_TO_RADIANS = 2 * Math.PI
                    / SWERVE_CONSTANTS.STEER_GEARING;

            SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND = 4.54;
            SWERVE_CONSTANTS.MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED = 8; // TODO
            SWERVE_CONSTANTS.MAX_STEER_VELOCITY_RADIANS_PER_SECOND = 100 * 2 * Math.PI; // TODO
            // max velocity in 1/3 sec
            SWERVE_CONSTANTS.MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 10 * 100 * 2 * Math.PI; // TODO

            SWERVE_CONSTANTS.DRIVE_CURRENT_LIMIT = 75;
            SWERVE_CONSTANTS.STEER_CURRENT_LIMIT = 30;
            SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR = DCMotor.getKrakenX60(1);
            SWERVE_CONSTANTS.STEER_GEARBOX_REPR = DCMotor.getKrakenX60(1);
            SWERVE_CONSTANTS.DRIVE_MOI = 0.01;
            SWERVE_CONSTANTS.STEER_MOI = 0.025;
        }

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 14.37;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 14.37 * 7; // TODO

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

        public static final double PATH_FOLLOWING_TRANSLATION_kP = 8.0; // TODO simvalue
        public static final double PATH_FOLLOWING_ROTATION_kP = 8.0; // TODO simvalue

        public static final double XY_kP = 1.4;
        public static final double XY_kI = 0;
        public static final double XY_kD = 0.05;
        public static final TrapezoidProfile.Constraints XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
                SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND,
                SWERVE_CONSTANTS.MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final double THETA_kP = 2.2; // TODO
        public static final double THETA_kI = 0; // TODO
        public static final double THETA_kD = 0.1; // TODO
        public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

    }

    public static final class Intake {
        // RANGE OF MOTION: 2.301

        // 2/15/24
        public static enum IntakePosition {
            GROUND(-0.617905),
            AMP(1.18),
            STOW(1.4),
            SOURCE(1.44),
            TOP(1.691);

            public final double value;

            private IntakePosition(double value) {
                this.value = value;
            }
        }

        public static final int PRIMARY_ARM_MOTOR_ID = 9;
        public static final int SECONDARY_ARM_MOTOR_ID = 10;
        public static final int ROLLER_MOTOR_ID = 13;

        public static final int SHOOTER_CLOSE_BEAM_ID = 0;
        public static final int SHOOTER_FAR_BEAM_ID = 1;
        public static final int INTAKE_BEAM_ID = 2;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNeoVortex(2);
        public static final double GEARING = 43.2;
        public static final double LENGTH_METERS = 0.23; // TODO simvalue
        public static final double MASS_KG = 4.082; // TODO simv<alue
        public static final double MOMENT_OF_INERTIA_KG_METERS_SQUARED = SingleJointedArmSim.estimateMOI(
                LENGTH_METERS,
                MASS_KG);

        public static final double MIN_ANGLE_RADIANS = -0.610;
        public static final double MAX_ANGLE_RADIANS = 1.691;

        public static final double ENCODER_ROTATIONS_TO_RADIANS = 2 * Math.PI / GEARING;
        public static final int ARM_CURRENT_LIMIT = 30;
        public static final int ROLLER_CURRENT_LIMIT = 65;

        // 3/28/24
        public static final double kP = 3.5;
        public static final double kI = 0.0;
        public static final double kD = 0.1;
        public static final double kS = 0.132468;
        public static final double kG = 0.273264;
        public static final double kV = 0.77622;
        public static final double kA = 0;
        public static final double TOLERANCE = 0.05;

        // max theoretical velocity: 15.777 rad/s
        // 2/14/24
        public static final double MAX_VELOCITY_RADIANS_PER_SECOND = 7;
        public static final double MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 14;
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_RADIANS_PER_SECOND, MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        public static final Pose3d BASE_COMPONENT_POSE = new Pose3d(-0.19, 0, 0.299,
                new Rotation3d(0, -Units.degreesToRadians(35), Math.PI));

    }

    public static final class Shooter {
        public static final int LEFT_MOTOR_ID = 11;
        public static final int RIGHT_MOTOR_ID = 12;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNeoVortex(2);
        public static final double GEARING = 1.0;
        public static final double WHEEL_AXLE_MASS = Units.lbsToKilograms(2.5);
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
        // 3lb, 2in radius, 1/2mr^2
        public static final double MOMENT_OF_INERTIA_KG_METERS_SQUARED = (1.0 / 2.0) * WHEEL_AXLE_MASS
                * Math.pow(WHEEL_RADIUS, 2);
        public static final int CURRENT_LIMIT = 70;

        public static final double IDLE_RPS = 35;
        public static final double SUBWOOFER_RPS = 55;
        public static final double PODIUM_RPS = 84;

        // 3/3/24
        public static final double left_kP = 0.1;
        public static final double left_kI = 0;
        public static final double left_kD = 0;
        public static final double left_kS = 0.14652;
        public static final double left_kV = 0.10797;
        public static final double left_kA = 0.022635;

        public static final double right_kP = 0.1;
        public static final double right_kI = 0;
        public static final double right_kD = 0;
        public static final double right_kS = 0.12047;
        public static final double right_kV = 0.10746;
        public static final double right_kA = 0.021566;
        public static final double TOLERANCE = 5;

        // public static final double MAX_VELOCITY_METERS_PER_SECOND = 80;
        // public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 10;
        // public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new
        // TrapezoidProfile.Constraints(
        // MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final double GOAL_POSITION_ITERATIONS = 5;
        public static final double ACCELERATION_COMPENSATION_FACTOR = 0.0;

        // key: distance, value: speed
        public static final InterpolatingDoubleTreeMap SPEED_INTERPOLATOR = new InterpolatingDoubleTreeMap();
        static {
            // 3/5/24
            SPEED_INTERPOLATOR.put(1.142, 45.0);
            SPEED_INTERPOLATOR.put(1.511, 55.0);
            SPEED_INTERPOLATOR.put(1.995, 78.0);
            SPEED_INTERPOLATOR.put(2.2845, 84.0);
            SPEED_INTERPOLATOR.put(2.593, 84.0);
            SPEED_INTERPOLATOR.put(2.87, 84.0);
            SPEED_INTERPOLATOR.put(3.128, 84.0);
            SPEED_INTERPOLATOR.put(3.478, 84.0);
            SPEED_INTERPOLATOR.put(3.834, 84.0);
            SPEED_INTERPOLATOR.put(4.239, 84.0);
            SPEED_INTERPOLATOR.put(4.597, 84.0);
            SPEED_INTERPOLATOR.put(5.1322, 84.0);
        }

        public static final double MAX_SHOOTING_DISTANCE = 5.1322;

        public static final double getProjectileSpeed(double shooterRps) {
            return shooterRps * 0.1446;
        }
    }

    public static final class ShooterTilt {
        // 0.160 max movement
        // 2/15/2024
        public static enum ShooterTiltPosition {
            BOTTOM(0.389842),
            AMP_EJECT(0.601),
            INTAKE(0.55),
            CLIMB(1.176781),
            SUBWOOFER(1.01572),
            PODIUM(0.651760);

            public final double value;

            private ShooterTiltPosition(double value) {
                this.value = value;
            }
        }

        public static final int MOTOR_ID = 14;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNeoVortex(1);
        public static final double GEARING = 5.0;
        public static final double MASS_KG = Units.lbsToKilograms(12);
        // 1 rot = 12mm
        public static final double ENCODER_ROTATIONS_TO_METERS = Units.inchesToMeters(0.5) / GEARING;

        public static final double MIN_HEIGHT_METERS = 0.005;
        public static final double MAX_HEIGHT_METERS = 0.14;

        public static final double MIN_ANGLE_RADIANS = 0.389842;
        public static final double MAX_ANGLE_RADIANS = 1.2;

        public static final int CURRENT_LIMIT = 25;

        // 3/28/24
        public static final double kP = 400;
        public static final double kI = 0;
        public static final double kD = 2;
        public static final double kS = 0.104904;
        public static final double kG = 0.124356;
        public static final double kV = 44.0148;
        public static final double kA = 3.48876;
        public static final double TOLERANCE = 0.02;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.3;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.32;
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final Pose3d BASE_SHOOTER_POSE = new Pose3d(-0.19, 0, 0.299, new Rotation3d(0, 0, 0));
        public static final Pose3d BASE_OUTER_LEAD_SCREW_POSE = new Pose3d(-0.0915,
                0, 0.125,
                new Rotation3d(0, 0, 0));

        public static final Transform3d OUTER_LEAD_SCREW_TO_INNER_LEAD_SCREW = new Transform3d(0.047, 0, 0.0127,
                new Rotation3d());
        public static final Transform3d LEAD_SCREW_PIVOT_TO_EXTENSION = new Transform3d(0, 0, 0.0127,
                new Rotation3d());
        public static final Transform3d SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT = new Transform3d(0.2572, 0,
                -0.0984,
                new Rotation3d());
        public static final double INITIAL_LEAD_SCREW_LENGTH = 0.2519;

        public static final Transform3d SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT = new Transform3d(0.09906, 0,
                -0.1737,
                new Rotation3d());

        public static final double getLinearActuatorLength(double angle) {
            double shooterToBottomLeadScrewAngle = Math
                    .atan(Math.abs(SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getX())
                            / Math.abs(SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getZ()));
            double shooterToTopLeadScrewAngle = Math
                    .atan(Math.abs(SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT.getZ())
                            / Math.abs(SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT.getX()));

            double shooterInteriorAngle = angle - shooterToBottomLeadScrewAngle - shooterToTopLeadScrewAngle
                    + Math.PI / 2.0;

            double shooterPivotToBottomPivot = SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getTranslation()
                    .getNorm(); // a
            double shooterPivotToTopPivot = SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT.getTranslation()
                    .getNorm(); // b

            return Math.sqrt(
                    Math.pow(shooterPivotToBottomPivot, 2)
                            - (2 * shooterPivotToBottomPivot * shooterPivotToTopPivot
                                    * Math.cos(shooterInteriorAngle))
                            + Math.pow(shooterPivotToTopPivot, 2)
                            - Math.pow(LEAD_SCREW_PIVOT_TO_EXTENSION.getZ(), 2))
                    - INITIAL_LEAD_SCREW_LENGTH;
        }

        // https://www.desmos.com/calculator/7ojeknrpiq
        public static final double getShooterAngle(double linearActuatorLength) {
            double fullLeadScrewLength = INITIAL_LEAD_SCREW_LENGTH + linearActuatorLength;

            double shooterPivotToBottomPivot = SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getTranslation()
                    .getNorm(); // a
            double shooterPivotToTopPivot = SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT.getTranslation()
                    .getNorm(); // b
            double leadScrewHyp = Math
                    .sqrt(Math.pow(LEAD_SCREW_PIVOT_TO_EXTENSION.getZ(), 2)
                            + Math.pow(fullLeadScrewLength, 2)); // c

            double shooterInteriorAngle = Math.acos(
                    (Math.pow(shooterPivotToBottomPivot, 2)
                            + Math.pow(shooterPivotToTopPivot, 2)
                            - Math.pow(leadScrewHyp, 2))
                            / (2 * shooterPivotToBottomPivot * shooterPivotToTopPivot));

            double shooterToBottomLeadScrewAngle = Math
                    .atan(Math.abs(SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getX())
                            / Math.abs(SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getZ()));
            double shooterToTopLeadScrewAngle = Math
                    .atan(Math.abs(SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT.getZ())
                            / Math.abs(SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT.getX()));

            return shooterInteriorAngle + shooterToBottomLeadScrewAngle + shooterToTopLeadScrewAngle
                    - Math.PI / 2.0;
        }

        // used only in simulation
        public static final double getLeadScrewAngle(double linearActuatorLength) {
            double fullLeadScrewLength = INITIAL_LEAD_SCREW_LENGTH + linearActuatorLength;

            double shooterPivotToBottomPivot = SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getTranslation()
                    .getNorm(); // a
            double shooterPivotToTopPivot = SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT.getTranslation()
                    .getNorm(); // b
            double leadScrewHyp = Math
                    .sqrt(Math.pow(LEAD_SCREW_PIVOT_TO_EXTENSION.getZ(), 2)
                            + Math.pow(fullLeadScrewLength, 2)); // c

            double leadScrewInteriorAngle = Math.acos(
                    (Math.pow(shooterPivotToBottomPivot, 2)
                            + Math.pow(leadScrewHyp, 2)
                            - Math.pow(shooterPivotToTopPivot, 2))
                            / (2 * shooterPivotToBottomPivot * leadScrewHyp));

            double bottomLeadScrewToShooterAngle = Math
                    .atan(Math.abs(SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getZ())
                            / Math.abs(SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getX()));

            return Math.PI - leadScrewInteriorAngle - bottomLeadScrewToShooterAngle;
        }

        // key: distance, value: angle
        public static final InterpolatingDoubleTreeMap ANGLE_INTERPOLATOR = new InterpolatingDoubleTreeMap();
        static {
            // 3/5/24
            ANGLE_INTERPOLATOR.put(1.1795122686667598, 1.0157209208113183);
            ANGLE_INTERPOLATOR.put(1.7062360399365777, 0.850934764294403);
            ANGLE_INTERPOLATOR.put(2.235756419155122, 0.7453064051470202);
            ANGLE_INTERPOLATOR.put(2.9124685062916904, 0.6297289478451282);
            ANGLE_INTERPOLATOR.put(3.419734229712845, 0.5651378496391);
            ANGLE_INTERPOLATOR.put(3.948809911586509, 0.544307740832874);
            ANGLE_INTERPOLATOR.put(4.239, 0.480486);
            ANGLE_INTERPOLATOR.put(4.597, 0.46);
            ANGLE_INTERPOLATOR.put(5.1322, 0.44);

        }
    }

    public static final class Climber {
        public static enum ClimberPosition {
            BOTTOM(0 + 1),
            ON_CHAIN(0.5144 + 1),
            CLIMB_PREP(0.605 + 1),
            MAX_HEIGHT(0.67485 + 1);

            public final double value;

            private ClimberPosition(double value) {
                this.value = value;
            }
        }

        public static final int MOTOR_ID = 15;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNeoVortex(1);
        public static final double GEARING = 36;
        public static final double CARRIAGE_MASS_KG = Units.lbsToKilograms(3);
        public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(0.75);
        public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * DRUM_RADIUS_METERS;
        public static final double ENCODER_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / GEARING;

        public static final double MIN_HEIGHT_METERS = 0 + 1;
        public static final double MAX_HEIGHT_METERS = 0.67485 + 1;

        public static final int CURRENT_LIMIT = 50;

        // 3/28/24
        public static final double kP = 20;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0.089874;
        public static final double kG = -0.0441588;
        public static final double kV = 31.89;
        public static final double kA = 1.68948;
        public static final double TOLERANCE = 0.01;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.28; // TODO
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.5; // TODO
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final Pose3d BASE_COMPONENT_POSE = new Pose3d(0.177, 0, 0.19, new Rotation3d(0, 0, 0));
    }

    public static final class NoteLift {
        public static enum NoteLiftPosition {
            BOTTOM(0),
            INTAKE(0.16),
            TRAVEL(0.4903106689453125),
            CLIMB_PREP(0.55),
            STOW(0.62),
            TOP(0.66);

            public final double value;

            private NoteLiftPosition(double value) {
                this.value = value;
            }
        }

        public static final int MOTOR_ID = 16;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNeoVortex(1);
        public static final double GEARING = 9;
        public static final double CARRIAGE_MASS_KG = Units.lbsToKilograms(3);
        public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(0.75);
        public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * DRUM_RADIUS_METERS;
        public static final double ENCODER_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / GEARING;

        public static final double MIN_HEIGHT_METERS = 0;
        public static final double MAX_HEIGHT_METERS = 0.69;

        public static final int CURRENT_LIMIT = 40;

        // 2/18/24
        public static final double kP = 30;
        public static final double kI = 0;
        public static final double kD = 1;
        public static final double kS = 0.362388;
        public static final double kG = -0.166392;
        public static final double kV = 8.18532;
        public static final double kA = 0.541548;
        public static final double TOLERANCE = 0.01;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.7; // TODO
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.7; // TODO
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final Pose3d BASE_COMPONENT_POSE = new Pose3d(0.177, 0, 0.20, new Rotation3d(0, 0, 0));
    }

    public static final class Vision {
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(Double.MAX_VALUE,
                Double.MAX_VALUE,
                Double.MAX_VALUE);
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.1, 0.1, Double.MAX_VALUE);

        public static final PhotonCameraConstants CAMERA_CONSTANTS = new PhotonCameraConstants();
        static {
            CAMERA_CONSTANTS.WIDTH = 1600;
            CAMERA_CONSTANTS.HEIGHT = 1200;
            CAMERA_CONSTANTS.FOV = 95.39;
            CAMERA_CONSTANTS.FPS = 35;
            CAMERA_CONSTANTS.AVG_LATENCY = 30;
            CAMERA_CONSTANTS.STDDEV_LATENCY = 15;
        }

        // 2/17/24
        public static final Transform3d[] ROBOT_TO_CAMS = new Transform3d[] {
                // front camera
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(11.886316),
                                -Units.inchesToMeters(7.507594),
                                Units.inchesToMeters(9.541569)),
                        new Rotation3d(0, Units.degreesToRadians(-25),
                                Units.degreesToRadians(10))),
                // left camera
                new Transform3d(
                        new Translation3d(
                                -Units.inchesToMeters(1.765373),
                                Units.inchesToMeters(10.707761),
                                Units.inchesToMeters(12.116848)),
                        new Rotation3d(0, Units.degreesToRadians(-20),
                                Units.degreesToRadians(70))),
                // right camera
                new Transform3d(
                        new Translation3d(
                                -Units.inchesToMeters(1.765373),
                                -Units.inchesToMeters(10.707761),
                                Units.inchesToMeters(12.116848)),
                        new Rotation3d(0, Units.degreesToRadians(-20),
                                Units.degreesToRadians(-70)))
        };
    }

    public static final class Teleop {
        /**
         * A value inputted into the rate limiter (the joystick input) can move from 0
         * to 1 in 1/RATE_LIMIT seconds.
         * 
         * A rate limit of 3, for example, means that 0->1 in 1/3 sec.
         * Larger numbers mean less of a rate limit.
         */
        public static final double JOYSTICK_INPUT_RATE_LIMIT = 15.0; // TODO
        public static final double JOYSTICK_INPUT_DEADBAND = 0.05; // TODO
        public static final double JOYSTICK_CURVE_EXP = 2; // TODO
        public static final double JOYSTICK_ROT_CURVE_EXP = 1; // TODO
    }

    public static final class LEDs {
        public static enum LEDSection implements BaseLEDSection {
            SHOOTER_RIGHT(0, 27),
            SHOOTER_TOP(28, 77),
            SHOOTER_LEFT(78, 110, true),
            SHOOTER(0, 110),
            SHOOTER_RIGHT_BOTTOM(0, 13),
            SHOOTER_RIGHT_TOP(14, 27),
            SHOOTER_LEFT_BOTTOM(94, 110, true),
            SHOOTER_LEFT_TOP(78, 93, true),

            ELEVATOR_LEFT(111, 221),
            ELEVATOR_LEFT_TOP(167, 221),
            ELEVATOR_LEFT_BOTTOM(111, 166),
            ELEVATOR_RIGHT(222, 332),
            ELEVATOR_RIGHT_TOP(278, 332),
            ELEVATOR_RIGHT_BOTTOM(222, 277),
            ALL(0, 332);

            private final int startIdx;
            private final int endIdx;
            private final boolean inverted;

            private LEDSection(int startIdx, int endIdx, boolean inverted) {
                this.startIdx = startIdx;
                this.endIdx = endIdx;
                this.inverted = inverted;
            }

            private LEDSection(int startIdx, int endIdx) {
                this(startIdx, endIdx, false);
            }

            @Override
            public int start() {
                return startIdx;
            }

            @Override
            public int end() {
                return endIdx;
            }

            @Override
            public boolean inverted() {
                return inverted;
            }

            @Override
            public int length() {
                return endIdx - startIdx + 1;
            }
        }

        public static final int PORT = 0;
        public static final int LENGTH = 333;
    }

    public static final class HoundBrian {
        public static final int BUTTON_1 = 3;
        public static final int BUTTON_2 = 4;
        public static final int BUTTON_3 = 5;
        public static final int BUTTON_4 = 6;
        public static final int BUTTON_5 = 7;
        public static final int BUTTON_6 = 8;
        public static final int BUTTON_7 = 9;
    }
}
