package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.techhounds.houndutil.houndlib.SparkConfigurator;
import com.techhounds.houndutil.houndlib.subsystems.BaseElevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Climber.*;

public class Climber extends SubsystemBase implements BaseElevator<ClimberPosition> {
    private ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, MOVEMENT_CONSTRAINTS);
    private ElevatorFeedforward feedforwardController = new ElevatorFeedforward(kS, kG, kV, kA);

    private double feedbackVoltage = 0;
    private double feedforwardVoltage = 0;

    private CANSparkFlex motor = SparkConfigurator.createSparkFlex(MOTOR_ID, MotorType.kBrushless,
            MOTOR_INVERTED, (motor) -> motor.setIdleMode(IdleMode.kBrake),
            (motor) -> motor.setSmartCurrentLimit(CURRENT_LIMIT),
            (motor) -> motor.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS),
            (motor) -> motor.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0));

    @Override
    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    @Override
    public void resetPosition() {
        motor.getEncoder().setPosition(0);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            feedbackVoltage = pidController.calculate(getPosition());
            feedforwardVoltage = feedforwardController.calculate(pidController.getSetpoint().position,
                    pidController.getSetpoint().velocity);
            setVoltage(feedbackVoltage + feedforwardVoltage);
        }).withName("Move to Current Goal");
    }

    @Override
    public Command moveToPositionCommand(Supplier<ClimberPosition> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get().position)),
                moveToCurrentGoalCommand().until(pidController::atGoal)).withName("Move to Position");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get())),
                moveToCurrentGoalCommand().until(pidController::atGoal)).withName("Move to Arbitrary Position");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> pidController.getGoal().position + delta.get())
                .withName("Move Position Delta");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> pidController.setGoal(getPosition())).andThen(moveToCurrentGoalCommand())
                .withName("Hold Current Position");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("Reset Position");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return run(() -> setVoltage(12.0 * speed.get())).withName("Set Overridden Speed");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> motor.stopMotor())
                .andThen(() -> {
                    motor.setIdleMode(IdleMode.kCoast);
                })
                .finallyDo((d) -> {
                    motor.setIdleMode(IdleMode.kBrake);
                    pidController.reset(getPosition());
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withName("Coast Motors");
    }

}