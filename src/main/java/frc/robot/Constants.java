package frc.robot;

import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera.PhotonCameraConstants;
import com.techhounds.houndutil.houndlib.leds.BaseLEDSection;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
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

    public static final class Drivetrain {
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
        public static final boolean STEER_CANCODERS_INVERTED = true;

        // TODO
        public static final double FRONT_LEFT_OFFSET = -0.001220703125;
        public static final double FRONT_RIGHT_OFFSET = 0.201171875;
        public static final double BACK_LEFT_OFFSET = 0.2403;
        public static final double BACK_RIGHT_OFFSET = -0.4331;

        /** Distance between left and right wheels. */
        public static final double TRACK_WIDTH_METERS = 0.52832;
        /** Distance between front and back wheels. */
        public static final double WHEEL_BASE_METERS = 0.52832;

        public static final SwerveConstants SWERVE_CONSTANTS = new SwerveConstants();
        static {
            SWERVE_CONSTANTS.DRIVE_kP = 2.6794 * (2.0 * Math.PI * 0.0478); // TODO simvalue
            SWERVE_CONSTANTS.DRIVE_kI = 0.0;
            SWERVE_CONSTANTS.DRIVE_kD = 0.0;
            SWERVE_CONSTANTS.DRIVE_kS = 0.0097954 * (2.0 * Math.PI * 0.0478); // TODO simvalue
            SWERVE_CONSTANTS.DRIVE_kV = 2.1268 * (2.0 * Math.PI * 0.0478);
            SWERVE_CONSTANTS.DRIVE_kA = 0.26277 * (2.0 * Math.PI * 0.0478);
            SWERVE_CONSTANTS.STEER_kP = 40.0; // TODO simvalue
            SWERVE_CONSTANTS.STEER_kI = 0.0;
            SWERVE_CONSTANTS.STEER_kD = 1.0; // TODO simvalue

            SWERVE_CONSTANTS.DRIVE_GEARING = 5.357;
            SWERVE_CONSTANTS.STEER_GEARING = 150.0 / 7.0;
            SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * 0.0478; // 0.3003362577
            SWERVE_CONSTANTS.DRIVE_ENCODER_ROTATIONS_TO_METERS = SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE
                    / SWERVE_CONSTANTS.DRIVE_GEARING;
            SWERVE_CONSTANTS.STEER_ENCODER_ROTATIONS_TO_RADIANS = 2 * Math.PI
                    / SWERVE_CONSTANTS.STEER_GEARING;

            // 64.4
            SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND = 5.583; // TODO simvalue
            SWERVE_CONSTANTS.MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED = 19.3416549959; // TODO simvalue
            SWERVE_CONSTANTS.MAX_STEER_VELOCITY_RADIANS_PER_SECOND = 100 * 2 * Math.PI; // TODO simvalue
            // max velocity in 1/3 sec
            SWERVE_CONSTANTS.MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 3 * 100 * 2 * Math.PI; // TODO simvalue

            SWERVE_CONSTANTS.DRIVE_CURRENT_LIMIT = 50; // TODO simvalue
            SWERVE_CONSTANTS.STEER_CURRENT_LIMIT = 20; // TODO simvalue
            SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR = DCMotor.getKrakenX60(1);
            SWERVE_CONSTANTS.STEER_GEARBOX_REPR = DCMotor.getKrakenX60(1);
            SWERVE_CONSTANTS.DRIVE_MOI = 0.04; // TODO simvalue
            SWERVE_CONSTANTS.STEER_MOI = 0.025; // TODO simvalue
        }

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 20 * Math.PI; // TODO
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 20 * Math.PI; // TODO

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

        public static final double XY_kP = 1; // TODO
        public static final double XY_kI = 0; // TODO
        public static final double XY_kD = 0; // TODO
        public static final TrapezoidProfile.Constraints XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
                SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND,
                SWERVE_CONSTANTS.MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final double THETA_kP = 1.5; // TODO
        public static final double THETA_kI = 0; // TODO
        public static final double THETA_kD = 0.1; // TODO
        public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        public static final double PATH_FOLLOWING_TRANSLATION_kP = 5; // TODO simvalue
        public static final double PATH_FOLLOWING_ROTATION_kP = 1; // TODO simvalue
    }

    public static final class Intake {
        public static enum IntakePosition {
            GROUND(Units.degreesToRadians(-35)), // TODO simvalue
            AMP(Units.degreesToRadians(60)), // TODO simvalue
            STOW(Units.degreesToRadians(95)); // TODO simvalue

            public final double value;

            private IntakePosition(double value) {
                this.value = value;
            }
        }

        public static final int PRIMARY_ARM_MOTOR_ID = 9;
        public static final int SECONDARY_ARM_MOTOR_ID = 10;
        public static final int ROLLER_MOTOR_ID = 11;

        public static final int INTAKE_BEAM_ID = 0;
        public static final int PRIMARY_SHOOTER_BEAM_ID = 1;
        public static final int SECONDARY_SHOOTER_BEAM_ID = 2;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNeoVortex(2);
        public static final double GEARING = 36.0;
        public static final double LENGTH_METERS = 0.23; // TODO simvalue
        public static final double MASS_KG = 4.082; // TODO simvalue
        public static final double MOMENT_OF_INERTIA_KG_METERS_SQUARED = SingleJointedArmSim.estimateMOI(
                LENGTH_METERS,
                MASS_KG);

        public static final double MIN_ANGLE_RADIANS = Units.degreesToRadians(-35); // TODO simvalue
        public static final double MAX_ANGLE_RADIANS = Units.degreesToRadians(100); // TODO simvalue

        public static final double ENCODER_ROTATIONS_TO_RADIANS = 2 * Math.PI / GEARING;
        public static final int ARM_CURRENT_LIMIT = 20; // TODO simvalue
        public static final int ROLLER_CURRENT_LIMIT = 10; // TODO simvalue

        public static final double kP = 33.03; // TODO simvalue
        public static final double kI = 0;
        public static final double kD = 0.834;
        public static final double kS = 0;
        public static final double kG = 0.21316; // TODO simvalue
        public static final double kV = 0.59658; // TODO simvalue
        public static final double kA = 0.019195; // TODO simvalue
        public static final double TOLERANCE = 0.01;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 7.5 * Math.PI; // TODO simvalue
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 7.5 * 5 * Math.PI; // TODO simvalue
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final Pose3d BASE_COMPONENT_POSE = new Pose3d(-0.19, 0, 0.299,
                new Rotation3d(0, -Units.degreesToRadians(35), Math.PI));

    }

    public static final class Shooter {
        public static final int PRIMARY_MOTOR_ID = 12;
        public static final int SECONDARY_MOTOR_ID = 13;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNeoVortex(2);
        public static final double GEARING = 1.0;
        public static final double WHEEL_AXLE_MASS = Units.lbsToKilograms(2.5);
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
        // 3lb, 2in radius, 1/2mr^2
        public static final double MOMENT_OF_INERTIA_KG_METERS_SQUARED = (1.0 / 2.0) * WHEEL_AXLE_MASS
                * Math.pow(WHEEL_RADIUS, 2);
        public static final int CURRENT_LIMIT = 40;

        public static final double IDLE_RPS = 8;
        public static final double SHOOTING_RPS = 70;

        public static final double kP = 0.5; // TODO simvalue
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0.011075; // TODO simvalue
        public static final double kV = 0.10431; // TODO simvalue
        public static final double kA = 0.036933; // TODO simvalue
        public static final double TOLERANCE = 0.05;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6600; // TODO simvalue
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1000; // TODO simvalue
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    }

    public static final class ShooterTilt {
        public static enum ShooterTiltPosition {
            BOTTOM(Units.degreesToRadians(25)), // TODO simvalue
            STOW(Units.degreesToRadians(80)); // TODO simvalue

            public final double value;

            private ShooterTiltPosition(double value) {
                this.value = value;
            }
        }

        public static final int MOTOR_ID = 14;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNeoVortex(1);
        public static final double GEARING = 16;
        public static final double MASS_KG = Units.lbsToKilograms(12); // TODO simvalue
        // 1 rot = 12mm
        public static final double ENCODER_ROTATIONS_TO_METERS = Units.inchesToMeters(0.5);

        public static final double MIN_HEIGHT_METERS = 0; // 9.921in // TODO simvalue
        public static final double MAX_HEIGHT_METERS = 0.1445; // TODO simvalue

        public static final double MIN_ANGLE_RADIANS = Units.degreesToRadians(22.0097); // TODO simvalue
        public static final double MAX_ANGLE_RADIANS = Units.degreesToRadians(72.4199); // TODO simvalue

        public static final int CURRENT_LIMIT = 10;

        public static final double kP = 1000; // TODO simvalue
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kG = 0.022466; // TODO simvalue
        public static final double kV = 131.43; // TODO simvalue
        public static final double kA = 6.5761; // TODO simvalue
        public static final double TOLERANCE = 0.01;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.1; // TODO simvalue
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.5; // TODO simvalue
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final Pose3d BASE_SHOOTER_POSE = new Pose3d(-0.19, 0, 0.299, new Rotation3d(0, 0, 0));
        // public static final Pose3d BASE_OUTER_LEAD_SCREW_POSE = new Pose3d(1, 1, 1,
        // new Rotation3d(0, 0, 0));
        public static final Pose3d BASE_OUTER_LEAD_SCREW_POSE = new Pose3d(-0.0915,
                0, 0.125,
                new Rotation3d(0, 0, 0));

        public static final Transform3d OUTER_LEAD_SCREW_TO_INNER_LEAD_SCREW = new Transform3d(0.047, 0, 0.0127,
                new Rotation3d());
        public static final Transform3d LEAD_SCREW_PIVOT_TO_EXTENSION = new Transform3d(0, 0, 0.0127, new Rotation3d());
        public static final Transform3d SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT = new Transform3d(0.2572, 0, -0.0984,
                new Rotation3d());
        public static final double INITIAL_LEAD_SCREW_LENGTH = 0.2519;

        public static final Transform3d SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT = new Transform3d(0.09906, 0, -0.1737,
                new Rotation3d());

        public static final double getLinearActuatorLength(double angle) {
            double shooterToBottomLeadScrewAngle = Math.atan(Math.abs(SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getX())
                    / Math.abs(SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getZ()));
            double shooterToTopLeadScrewAngle = Math.atan(Math.abs(SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT.getZ())
                    / Math.abs(SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT.getX()));

            double shooterInteriorAngle = angle - shooterToBottomLeadScrewAngle - shooterToTopLeadScrewAngle
                    + Math.PI / 2.0;

            double shooterPivotToBottomPivot = SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getTranslation().getNorm(); // a
            double shooterPivotToTopPivot = SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT.getTranslation().getNorm(); // b

            return Math.sqrt(
                    Math.pow(shooterPivotToBottomPivot, 2)
                            - (2 * shooterPivotToBottomPivot * shooterPivotToTopPivot * Math.cos(shooterInteriorAngle))
                            + Math.pow(shooterPivotToTopPivot, 2) - Math.pow(LEAD_SCREW_PIVOT_TO_EXTENSION.getZ(), 2))
                    - INITIAL_LEAD_SCREW_LENGTH;
        }

        // https://www.desmos.com/calculator/7ojeknrpiq
        public static final double getShooterAngle(double linearActuatorLength) {
            double fullLeadScrewLength = INITIAL_LEAD_SCREW_LENGTH + linearActuatorLength;

            double shooterPivotToBottomPivot = SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getTranslation().getNorm(); // a
            double shooterPivotToTopPivot = SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT.getTranslation().getNorm(); // b
            double leadScrewHyp = Math
                    .sqrt(Math.pow(LEAD_SCREW_PIVOT_TO_EXTENSION.getZ(), 2) + Math.pow(fullLeadScrewLength, 2)); // c

            double shooterInteriorAngle = Math.acos(
                    (Math.pow(shooterPivotToBottomPivot, 2)
                            + Math.pow(shooterPivotToTopPivot, 2)
                            - Math.pow(leadScrewHyp, 2))
                            / (2 * shooterPivotToBottomPivot * shooterPivotToTopPivot));

            double shooterToBottomLeadScrewAngle = Math.atan(Math.abs(SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getX())
                    / Math.abs(SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getZ()));
            double shooterToTopLeadScrewAngle = Math.atan(Math.abs(SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT.getZ())
                    / Math.abs(SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT.getX()));

            return shooterInteriorAngle + shooterToBottomLeadScrewAngle + shooterToTopLeadScrewAngle - Math.PI / 2.0;
        }

        // used only in simulation
        public static final double getLeadScrewAngle(double linearActuatorLength) {
            double fullLeadScrewLength = INITIAL_LEAD_SCREW_LENGTH + linearActuatorLength;

            double shooterPivotToBottomPivot = SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getTranslation().getNorm(); // a
            double shooterPivotToTopPivot = SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT.getTranslation().getNorm(); // b
            double leadScrewHyp = Math
                    .sqrt(Math.pow(LEAD_SCREW_PIVOT_TO_EXTENSION.getZ(), 2) + Math.pow(fullLeadScrewLength, 2)); // c

            double leadScrewInteriorAngle = Math.acos(
                    (Math.pow(shooterPivotToBottomPivot, 2)
                            + Math.pow(leadScrewHyp, 2)
                            - Math.pow(shooterPivotToTopPivot, 2))
                            / (2 * shooterPivotToBottomPivot * leadScrewHyp));

            double bottomLeadScrewToShooterAngle = Math.atan(Math.abs(SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getZ())
                    / Math.abs(SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getX()));

            return Math.PI - leadScrewInteriorAngle - bottomLeadScrewToShooterAngle;
        }

        // TODO i need to actually make sure these are right
        public static final double LEAD_SCREW_RADIUS_METERS = Units.inchesToMeters(0.613); // ?
        public static final double SHOOTER_PIVOT_RADIUS_METERS = Units.inchesToMeters(1.625); // ?
        public static final double SHOOTER_OFFSET_ANGLE_RADIANS = 1337;
        public static final double ANGLE_ALPHA = 1337; // ?

        public static final double BOTTOM_PIVOT_TO_TOP_PIVOT_LENGTH_METERS = Units.inchesToMeters(7.843);
        public static final double SHOOTER_PIVOT_TO_ENDPOINT_PIVOT_LENGTH_METERS = Units.inchesToMeters(10.725);

        public static final double LEAD_SCREW_MIN_LENGTH_METERS = 1337;
        public static final double HORIZONTAL_ANGLE_OFFSET_RADIANS = 1337;

        /*
         * By treating the shooter + shooter tilt as a triangle with vertices at the 3
         * pivot points, we can use trig to get the shooter's approximate angle based on
         * lead screw length
         * units are radians and meters
         */

        public double getAngleFromLength(double length) {
            length += LEAD_SCREW_MIN_LENGTH_METERS;
            double theta = Math.acos((LEAD_SCREW_RADIUS_METERS * LEAD_SCREW_RADIUS_METERS +
                    length * length
                    - SHOOTER_PIVOT_TO_ENDPOINT_PIVOT_LENGTH_METERS * SHOOTER_PIVOT_TO_ENDPOINT_PIVOT_LENGTH_METERS -
                    BOTTOM_PIVOT_TO_TOP_PIVOT_LENGTH_METERS * BOTTOM_PIVOT_TO_TOP_PIVOT_LENGTH_METERS)
                    / (-2 * SHOOTER_PIVOT_TO_ENDPOINT_PIVOT_LENGTH_METERS * BOTTOM_PIVOT_TO_TOP_PIVOT_LENGTH_METERS));
            theta += HORIZONTAL_ANGLE_OFFSET_RADIANS; // to include or not to include...
            return theta;
        }

        /*
         * The inverse equation of the method above
         * units are meters and radians
         */
        public double getLengthFromAngle(double angle) {
            angle += HORIZONTAL_ANGLE_OFFSET_RADIANS; // angle to the horizontal + that offset that we need
            double length = Math.sqrt(-2 * BOTTOM_PIVOT_TO_TOP_PIVOT_LENGTH_METERS
                    * SHOOTER_PIVOT_TO_ENDPOINT_PIVOT_LENGTH_METERS * Math.cos(angle) +
                    BOTTOM_PIVOT_TO_TOP_PIVOT_LENGTH_METERS * BOTTOM_PIVOT_TO_TOP_PIVOT_LENGTH_METERS +
                    SHOOTER_PIVOT_TO_ENDPOINT_PIVOT_LENGTH_METERS * SHOOTER_PIVOT_TO_ENDPOINT_PIVOT_LENGTH_METERS -
                    LEAD_SCREW_RADIUS_METERS * LEAD_SCREW_RADIUS_METERS);
            length -= LEAD_SCREW_MIN_LENGTH_METERS; // include or...
            return length;
        }

        public double aimingRegression(double angle) {
            // there's going to be a regression function here once we test the shooter
            double fudgeFactor = 65537;
            return fudgeFactor;
        }

        /*
         * this does effectively nothing except for account for miniscule positional
         * changes in the entry point of the shooter, but it was fun to figure this out
         * so im going to keep this in
         * dx and dy are the target's horizontal and vertical distances from the shooter
         * pivot
         * units in meters and radians
         */
        public double mathematicallyPerfectLengthFromTarget(double dx, double dy) {

            double h = 2 * Math.atan(
                    (dy - Math.sqrt(dx * dx + dy * dy - SHOOTER_PIVOT_RADIUS_METERS * SHOOTER_PIVOT_RADIUS_METERS))
                            / (dx + SHOOTER_PIVOT_RADIUS_METERS));
            double desiredAngle = ANGLE_ALPHA - h + SHOOTER_OFFSET_ANGLE_RADIANS;
            return getLengthFromAngle(desiredAngle);
        }
    }

    public static final class Climber {
        public static enum ClimberPosition {
            BOTTOM(0), // TODO
            TOP(0.5); // TODO

            public final double value;

            private ClimberPosition(double value) {
                this.value = value;
            }
        }

        public static final int MOTOR_ID = 15;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNeoVortex(1);
        public static final double GEARING = 20;
        public static final double CARRIAGE_MASS_KG = Units.lbsToKilograms(3);
        public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(0.75);
        public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * DRUM_RADIUS_METERS;
        public static final double ENCODER_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / GEARING;

        public static final double MIN_HEIGHT_METERS = 0; // TODO ask nate
        public static final double MAX_HEIGHT_METERS = 0.67; // TODO ask nate

        public static final int CURRENT_LIMIT = 10;

        public static final double kP = 0; // TODO
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kG = 0.062861; // TODO
        public static final double kV = 78.237; // TODO
        public static final double kA = 5.0061; // TODO
        public static final double TOLERANCE = 0.01;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.152; // TODO
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.152 * 10; // TODO
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final Pose3d BASE_COMPONENT_POSE = new Pose3d(0.177, 0, 0.198, new Rotation3d(0, 0, 0));
    }

    public static final class NoteLift {
        public static enum NoteLiftPosition {
            BOTTOM(0), // TODO
            TOP(0.5); // TODO

            public final double value;

            private NoteLiftPosition(double value) {
                this.value = value;
            }
        }

        public static final int MOTOR_ID = 16;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNeoVortex(1);
        public static final double GEARING = 20;
        public static final double CARRIAGE_MASS_KG = Units.lbsToKilograms(3);
        public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(0.75);
        public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * DRUM_RADIUS_METERS;
        public static final double ENCODER_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / GEARING;

        public static final double MIN_HEIGHT_METERS = 0; // TODO ask nate
        public static final double MAX_HEIGHT_METERS = 0.67; // TODO ask nate

        public static final int CURRENT_LIMIT = 10;

        public static final double kP = 0; // TODO
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kG = 0.062861; // TODO
        public static final double kV = 78.237; // TODO
        public static final double kA = 5.0061; // TODO
        public static final double TOLERANCE = 0.01;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.152; // TODO
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.152 * 10; // TODO
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final Pose3d BASE_COMPONENT_POSE = new Pose3d(0.177, 0, 0.2, new Rotation3d(0, 0, 0));
    }

    public static final class Vision {
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(6, 6, 12);
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

        public static final PhotonCameraConstants CAMERA_CONSTANTS = new PhotonCameraConstants();
        static {
            CAMERA_CONSTANTS.WIDTH = 1600;
            CAMERA_CONSTANTS.HEIGHT = 1200;
            CAMERA_CONSTANTS.FOV = 95.39;
            CAMERA_CONSTANTS.FPS = 35;
            CAMERA_CONSTANTS.AVG_LATENCY = 30;
            CAMERA_CONSTANTS.STDDEV_LATENCY = 15;
        }

        // TODO
        public static final Transform3d[] ROBOT_TO_CAMS = new Transform3d[] {
                new Transform3d(
                        new Translation3d(0.26416, 0.26416, 0.25),
                        new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(-10))),
                new Transform3d(
                        new Translation3d(-0.26416, 0.26416, 0.25),
                        new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(70))),
                new Transform3d(
                        new Translation3d(-0.26416, -0.26416, 0.25),
                        new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(-70)))
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
    }

    public static final class LEDs {
        public static enum LEDSection implements BaseLEDSection {
            SHOOTER_RIGHT(0, 10),
            SHOOTER_TOP(0, 0),
            SHOOTER_LEFT(0, 0, true),
            ELEVATOR_LEFT(0, 0),
            ELEVATOR_RIGHT(0, 0);

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
            public int getStart() {
                return startIdx;
            }

            @Override
            public int getEnd() {
                return endIdx;
            }

            @Override
            public boolean getInverted() {
                return inverted;
            }
        }

        public static final int PORT = 0;
        public static final int LENGTH = 100; // TODO
    }
}
