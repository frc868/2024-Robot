package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class ShooterTilt {

        public static enum ShooterTiltPosition {
            TEMP(1337),
            BOTTOM(1337);

            public final double value;

            private ShooterTiltPosition(double value) {
                this.value = value;
            }
        }

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

        public static final int MOTOR_ID = 1337;

        public static final double ENCODER_ROTATIONS_TO_METERS = 1337;

        public static final int CURRENT_LIMIT = 1337;
        public static final double TOLERANCE = 1337;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 1337;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1337;
        public static final TrapezoidProfile.Constraints NORMAL_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final double MAX_STOWING_VELOCITY_METERS_PER_SECOND = 1337;
        public static final double MAX_STOWING_ACCELERATION_METERS_PER_SECOND_SQUARED = 1337;
        public static final TrapezoidProfile.Constraints STOWING_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_STOWING_VELOCITY_METERS_PER_SECOND, MAX_STOWING_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final double kP = 1337;
        public static final double kI = 1337;
        public static final double kD = 1337;

        public static final double kS = 1337;
        public static final double kG = 1337;
        public static final double kV = 1337;
        public static final double kA = 1337;

        // geometry
        public static final double MAX_LENGTH_METERS = 1337; // ?
        public static final double MIN_LENGTH_METERS = 1337;

        // TODO i need to actually make sure these are right
        public static final double LEAD_SCREW_RADIUS_METERS = Units.inchesToMeters(0.613); // ?
        public static final double SHOOTER_PIVOT_RADIUS_METERS = Units.inchesToMeters(1.625); // ?
        public static final double SHOOTER_OFFSET_ANGLE_RADIANS = 1337;
        public static final double ANGLE_ALPHA = 1337; // ?

        public static final double BOTTOM_PIVOT_TO_TOP_PIVOT_LENGTH_METERS = Units.inchesToMeters(7.843);
        public static final double SHOOTER_PIVOT_TO_ENDPOINT_PIVOT_LENGTH_METERS = Units.inchesToMeters(10.725);

        public static final double LEAD_SCREW_MIN_LENGTH_METERS = 1337;
        public static final double HORIZONTAL_ANGLE_OFFSET_RADIANS = 1337;

    }
}
