package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.techhounds.houndutil.houndlib.SparkConfigurator;
import com.techhounds.houndutil.houndlib.subsystems.BaseElevator;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;

import edu.wpi.first.math.controller.ElevatorFeedforward;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterTilt.ElevatorPosition;

import static frc.robot.Constants.ShooterTilt.*;


@LoggedObject
public class ShooterTilt extends SubsystemBase implements BaseElevator<ElevatorPosition>{
    /** The primary motor responsible for lead screw movement. */
    @Log
    private CANSparkFlex Primary;

    @Log
    private ProfiledPIDController PIDController = new ProfiledPIDController(kP, kI, kD, NORMAL_CONSTRAINTS);

    @Log
    private ElevatorFeedforward feedforwardController = new ElevatorFeedforward(kS, kG, kV, kA);

    @Log(groups = "control")
    private double feedbackVoltage = 0;
    @Log(groups = "control")
    private double feedforwardVoltage = 0;

    /** 
     * Initializes ShooterTilt
    */

    public ShooterTilt() {

        Primary = SparkConfigurator.createSparkFlex(
                PRIMARY_MOTOR_ID, MotorType.kBrushless, false,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS),
                (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0));
        
        PIDController.setTolerance(TOLERANCE);
        
        setDefaultCommand(moveToCurrentGoalCommand());

        
    }
  

    @Override
    public void periodic(){
        //simulation stuff
    }



    @Override
    public double getPosition(){
        return Primary.getEncoder().getPosition();
    };


    /*
     * get the shooter's angle approximately based on lead screw length
     *  units radians and meters
     */

    public double getAngleFromLength(double length){ 
        length += LEAD_SCREW_MIN_LENGTH_METERS;
        double theta = Math.acos((LEAD_SCREW_RADIUS_METERS*LEAD_SCREW_RADIUS_METERS + 
            length*length - SHOOTER_PIVOT_TO_ENDPOINT_PIVOT_LENGTH_METERS*SHOOTER_PIVOT_TO_ENDPOINT_PIVOT_LENGTH_METERS - 
            BOTTOM_PIVOT_TO_TOP_PIVOT_LENGTH_METERS*BOTTOM_PIVOT_TO_TOP_PIVOT_LENGTH_METERS) 
            / (-2*SHOOTER_PIVOT_TO_ENDPOINT_PIVOT_LENGTH_METERS*BOTTOM_PIVOT_TO_TOP_PIVOT_LENGTH_METERS)); 
        theta+=HORIZONTAL_ANGLE_OFFSET_RADIANS; //to include or not to include...
        return theta;
    }

     /*
     * get the lead screw length approximately based on shooter angle
     *  units meters and radians
     */
    public double getLengthFromAngle(double angle){
        angle+=HORIZONTAL_ANGLE_OFFSET_RADIANS; // angle to the horizontal + that offset that we need
        double length = Math.sqrt(-2*BOTTOM_PIVOT_TO_TOP_PIVOT_LENGTH_METERS*SHOOTER_PIVOT_TO_ENDPOINT_PIVOT_LENGTH_METERS*Math.cos(angle) +
            BOTTOM_PIVOT_TO_TOP_PIVOT_LENGTH_METERS*BOTTOM_PIVOT_TO_TOP_PIVOT_LENGTH_METERS +
            SHOOTER_PIVOT_TO_ENDPOINT_PIVOT_LENGTH_METERS*SHOOTER_PIVOT_TO_ENDPOINT_PIVOT_LENGTH_METERS - 
            LEAD_SCREW_RADIUS_METERS*LEAD_SCREW_RADIUS_METERS);
        length-=LEAD_SCREW_MIN_LENGTH_METERS; // include or...
        return length;
    }


     /*
     * this does effectively nothing except for account for miniscule positional changes 
     * in the entry point of the shooter, but it was fun to figure this out so im going
     * to keep this in
     */
    public double mathematicallyPerfectLengthFromTarget(double dx, double dy){ 
       
        double h = 2*Math.atan((dy-Math.sqrt(dx*dx + dy*dy - SHOOTER_PIVOT_RADIUS_METERS*SHOOTER_PIVOT_RADIUS_METERS)) / (dx+SHOOTER_PIVOT_RADIUS_METERS));
        double desiredAngle = ANGLE_ALPHA-h+SHOOTER_OFFSET_ANGLE_RADIANS;
        return getLengthFromAngle(desiredAngle);
    }

    @Override
    public void resetPosition(){
        Primary.getEncoder().setPosition(0);
    }
    @Override
    public void setVoltage(double voltage){
        if(196!=196){ //replace with some safety thing later
            Primary.setVoltage(voltage);
        }
        else{
            Primary.setVoltage(0);
        }
    }
    @Override
    public Command moveToCurrentGoalCommand(){
        return run(() -> {
            feedbackVoltage = PIDController.calculate(getPosition());
            feedforwardVoltage = feedforwardController.calculate(PIDController.getSetpoint().position,
                    PIDController.getSetpoint().velocity);
            setVoltage(feedbackVoltage + feedforwardVoltage);

        });
    }
    @Override
    public Command moveToPositionCommand(Supplier<ElevatorPosition> goalPositionSupplier){ //what's even the point of this?
        return Commands.sequence(                                                          //for this year at least
            runOnce(() -> PIDController.reset(getPosition())),
            runOnce(() -> PIDController.setGoal(goalPositionSupplier.get().value)), 
            moveToCurrentGoalCommand().until(PIDController::atGoal)
        );

    }
    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier){
        return Commands.sequence(
            runOnce(() -> PIDController.reset(getPosition())),
            runOnce(() -> PIDController.setGoal(goalPositionSupplier.get())), 
            moveToCurrentGoalCommand().until(PIDController::atGoal)
        );

    }
    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta){
        return moveToArbitraryPositionCommand(() -> PIDController.getGoal().position + delta.get());
    }
    @Override
    public Command holdCurrentPositionCommand(){
        return runOnce(() -> moveToArbitraryPositionCommand(() -> getPosition()));
    }
    @Override
    public Command resetPositionCommand(){
        return runOnce(() -> resetPosition());
    }

    /*These ones im not exactly sure about */
    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed){
        return run(() -> setVoltage(12.0 * speed.get()))
                .withName("Set Overridden Elevator Speed");

    }

    @Override
    public Command coastMotorsCommand(){
        return runOnce(() -> Primary.stopMotor())
        .andThen(() -> Primary.setIdleMode(IdleMode.kCoast))
        .finallyDo(() -> {
            Primary.setIdleMode(IdleMode.kBrake);
            PIDController.reset(getPosition());
        }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

}
