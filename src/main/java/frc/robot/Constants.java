package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
    
    public static final class ShooterTilt {
        
        public static enum ElevatorPosition {
            TEMP(1337);

            public final double value;
            private ElevatorPosition(double value) {
                this.value = value;
            }
        }

        

        public static final int PRIMARY_MOTOR_ID = 1337;

        public static final double ENCODER_ROTATIONS_TO_METERS = 1337;

        public static final int CURRENT_LIMIT = 1337;
        public static final double TOLERANCE = 1337;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 1337;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED=1337; 
        public static final TrapezoidProfile.Constraints NORMAL_CONSTRAINTS = new TrapezoidProfile.Constraints( MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED); 
        
        public static final double kP = 1337;
        public static final double kI = 1337;
        public static final double kD = 1337;
        
        public static final double kS = 1337;
        public static final double kG = 1337;
        public static final double kV = 1337;
        public static final double kA = 1337;



        //geometry
        public static final double MAX_LENGTH_METERS = 1337;
        public static final double MIN_LENGTH_METERS = 1337;

        //TODO i need to actually make sure these are right
        public static final double LEAD_SCREW_RADIUS_METERS = 0.613*0.0254;
        public static final double SHOOTER_PIVOT_RADIUS_METERS = 1.625*0.0254; //not useful
        public static final double SHOOTER_OFFSET_ANGLE_RADIANS = 1337;
        public static final double ANGLE_ALPHA = 1337; //not useful

        public static final double BOTTOM_PIVOT_TO_TOP_PIVOT_LENGTH_METERS = 7.843*0.0254;
        public static final double SHOOTER_PIVOT_TO_ENDPOINT_PIVOT_LENGTH_METERS =  10.725*0.0254;
        
        public static final double ENDPOINT_PIVOT_TO_ENDPOINT_DX = 1337;
        public static final double ENDPOINT_PIVOT_TO_ENDPOINT_DY = 1337;
        public static final double SHOOTER_PIVOT_TO_ENDPOINT_ANGLE = Math.atan(ENDPOINT_PIVOT_TO_ENDPOINT_DY/(SHOOTER_PIVOT_TO_ENDPOINT_PIVOT_LENGTH_METERS+ENDPOINT_PIVOT_TO_ENDPOINT_DX));
        public static final double LEAD_SCREW_MIN_LENGTH_METERS = 1337;
        public static final double HORIZONTAL_ANGLE_OFFSET_RADIANS = 1337;


        

    }
}
