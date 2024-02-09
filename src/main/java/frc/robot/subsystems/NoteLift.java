package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.techhounds.houndutil.houndlib.SparkConfigurator;
import com.techhounds.houndutil.houndlib.subsystems.BaseElevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NoteLift.NoteLiftPosition;

import static frc.robot.Constants.NoteLift.*;

import java.util.function.Supplier;

public class NoteLift  extends SubsystemBase implements BaseElevator {
    
    private ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, MOVEMENT_CONSTRAINTS);
    private ElevatorFeedforward feedforwardController = new ElevatorFeedforward(kS, kG, kV, kA);

    private double feedbackVoltage = 0;
    private double feedforwardVoltage = 0;

    private CANSparkFlex noteLiftMotor = SparkConfigurator.createSparkFlex(NOTE_LIFT_MOTOR_ID, MotorType.kBrushless,
            false);

    @Override
    public double getPosition() {
        return noteLiftMotor.getEncoder().getPosition();
    }

    @Override
    public void resetPosition() {
        noteLiftMotor.getEncoder().setPosition(0);
    }

    @Override
    public void setVoltage(double voltage) {
        noteLiftMotor.setVoltage(voltage);
    }

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
    public Command moveToPositionCommand(Supplier<NoteLiftPosition> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get().position)),
                moveToCurrentGoalCommand().until(pidController::atGoal));
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier goalPositionSupplier) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'moveToArbitraryPositionCommand'");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier delta) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'movePositionDeltaCommand'");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'holdCurrentPositionCommand'");
    }

    @Override
    public Command resetPositionCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetPositionCommand'");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier speed) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setOverridenSpeedCommand'");
    }

    @Override
    public Command coastMotorsCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'coastMotorsCommand'");
    }

}
