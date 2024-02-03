package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
    public static final class Climber {
        public static final int CLIMBER_MOTOR_ID = 0;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 0;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0;

        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    }
}
