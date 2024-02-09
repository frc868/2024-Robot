package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
    public static final class Climber {
        public static enum ClimberPosition {
            TOP(0), // untested
            BOTTOM(0); // untested

            public final double position;

            private ClimberPosition(double position) {
                this.position = position;
            }
        }

        public static final int CLIMBER_MOTOR_ID = 0;

        public static final double kP = 0; // untested
        public static final double kI = 0; // untested
        public static final double kD = 0; // untested
        public static final double kS = 0; // untested
        public static final double kG = 0; // untested
        public static final double kV = 0; // untested
        public static final double kA = 0; // untested

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 0; // untested
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0; // untested

        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    }

    public static final class NoteLift {
        public static enum NoteLiftPosition {
            TOP(0), // untested
            BOTTOM(0); // untested

            public final double position;

            private NoteLiftPosition(double position) {
                this.position = position;
            }
        }

        public static final int NOTE_LIFT_MOTOR_ID = 0;

        public static final double kP = 0; // untested
        public static final double kI = 0; // untested
        public static final double kD = 0; // untested
        public static final double kS = 0; // untested
        public static final double kG = 0; // untested
        public static final double kV = 0; // untested
        public static final double kA = 0; // untested

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 0; // untested
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0; // untested

        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    }
}
