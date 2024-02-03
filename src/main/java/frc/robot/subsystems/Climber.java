package frc.robot.subsystems;

import static frc.robot.Constants.Climber.CLIMBER_MOTOR_ID;

import java.util.function.Supplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.techhounds.houndutil.houndlib.subsystems.BaseElevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase implements BaseElevator {

    private ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD);
    private ElevatorFeedforward feedforwardController = new ElevatorFeedforward(kS, kG, kV, kA);

    private double feedbackVoltage = 0;
    private double feedforwardVoltage = 0;

    private CANSparkFlex climberMotor = new CANSparkFlex(CLIMBER_MOTOR_ID, MotorType.kBrushless);
    // private CANSparkFlex noteLiftMotor = new
    // CANSparkFlex(Constants.CAN.ELEVATOR_LEFT_MOTOR_ID, MotorType.kBrushless);

    @Override
    public double getPosition() {
        return climberMotor.getEncoder().getPosition();
    }

    @Override
    public void resetPosition() {
        climberMotor.getEncoder().setPosition(0);
    }

    @Override
    public void setVoltage(double voltage) {
        climberMotor.setVoltage(voltage);
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        return run(()->{
            feedbackVoltage = pidController.calculate(getPosition());
            feedforwardVoltage = feedforwardController.calculate(pidController.getSetpoint().position, pidController.getSetpoint().velocity);
            setVoltage(feedbackVoltage + feedforwardVoltage);
        })
    }

    @Override
    public Command moveToPositionCommand(Supplier<ElevatorPosition> goalPositionSupplier) {
        // TODO
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier goalPositionSupplier) {
        return Commands.sequence{
            runOnce(()-> pidController.reset(getPosition())),
            runOnce(()-> pidController.setGoal(goalPositionSupplier.get())),
            moveToCurrentGoalCommand().until(pidController::atGoal)
        }
    }

    @Override
    public Command movePositionDeltaCommand(Supplier delta) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'movePositionDeltaCommand'");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> pidController.setGoal(getPosition())).andThen(moveToCurrentGoalCommand());
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition);
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier speed) {
        return run(() -> setVoltage(12.0 * speed.get()));
    }

    @Override
    public Command coastMotorsCommand() {

    }

}
