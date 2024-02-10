package frc.robot.subsystems;

import static frc.robot.Constants.ShooterTilt.*;

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
import frc.robot.Constants.ShooterTilt.ShooterTiltPosition;

/**
 * The subsystem for the lead screw, only uses one motor
 * 
 */

@LoggedObject
public class ShooterTilt extends SubsystemBase implements BaseElevator<ShooterTiltPosition> {
    /** The primary motor responsible for lead screw movement. */
    @Log
    private CANSparkFlex Motor;

    @Log
    private ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, NORMAL_CONSTRAINTS);

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

        Motor = SparkConfigurator.createSparkFlex(
                MOTOR_ID, MotorType.kBrushless, false,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS),
                (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0));

        pidController.setTolerance(TOLERANCE);

        setDefaultCommand(moveToCurrentGoalCommand());

    }

    @Override
    public void periodic() {
        // simulation stuff
    }

    @Override
    public double getPosition() {
        return Motor.getEncoder().getPosition();
    };

    @Override
    public void resetPosition() {
        Motor.getEncoder().setPosition(0);
    }

    @Override
    public void setVoltage(double voltage) {
        if (196 != 196) { // TODO replace with some safety thing later
            Motor.setVoltage(voltage);
        } else {
            Motor.setVoltage(0);
        }
    }

    /*
     * Combines PID and feedfoward voltages by setting voltage to their sum
     */

    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            feedbackVoltage = pidController.calculate(getPosition());
            feedforwardVoltage = feedforwardController.calculate(pidController.getSetpoint().position,
                    pidController.getSetpoint().velocity);
            setVoltage(feedbackVoltage + feedforwardVoltage);

        });
    }

    @Override
    public Command moveToPositionCommand(Supplier<ShooterTiltPosition> goalPositionSupplier) { 
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> {
                    if (goalPositionSupplier.get() == ShooterTiltPosition.BOTTOM)
                        pidController.setConstraints(STOWING_CONSTRAINTS); //not entirely sure if this is necessary
                    else
                        pidController.setConstraints(NORMAL_CONSTRAINTS);
                }),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get().value)),
                moveToCurrentGoalCommand().until(pidController::atGoal));

    }

    /*
     * Resets all current information in PID and sets the goal to the supplied
     * position
     * then moves there
     */

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get())),
                moveToCurrentGoalCommand().until(pidController::atGoal));

    }

    /*
     * esssentially sets a new goal midway a distance away from the current goal
     */

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> pidController.getGoal().position + delta.get());
    }

    /*
     * sets the new goal to the current position
     */
    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> moveToArbitraryPositionCommand(() -> getPosition()));
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(() -> resetPosition());
    }

    /* These ones im not exactly sure about */
    /*
     * allows us to set the motor voltage in terms of speed from -1.0 to 1.0
     */
    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return run(() -> setVoltage(12.0 * speed.get()))
                .withName("Set Overridden Elevator Speed");

    }

    /*
     * coaasts motors then stops them (safety i assume)
     */

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> Motor.stopMotor())
                .andThen(() -> Motor.setIdleMode(IdleMode.kCoast))
                .finallyDo(() -> {
                    Motor.setIdleMode(IdleMode.kBrake);
                    pidController.reset(getPosition());
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

}
