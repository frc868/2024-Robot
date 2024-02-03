package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;

import com.techhounds.houndutil.houndlib.SparkConfigurator;
import com.techhounds.houndutil.houndlib.subsystems.BaseShooter;
import com.techhounds.houndutil.houndlog.interfaces.Log;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.constants.Shooter.*;

public class shooter extends SubsystemBase implements BaseShooter {

    //example motor
    private CANSparkFlex Left;
    private CANSparkFlex Right;
    //example velocity variable

    private SimpleMotorFeedforward feedforwardController = new SimpleMotorFeedforward(kS, kV, kA);
    private PIDController PIDController = new PIDController(kP,kI,kD);

    @Log(groups = "control")
    private double feedbackVoltage = 0;
    @Log(groups = "control")
    private double feedforwardVoltage = 0;


    public void Shooter(){
        Left = SparkConfigurator.createSparkFlex(
                LEFT_MOTOR_ID, MotorType.kBrushless, false,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS),
                (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0));

        Right = SparkConfigurator.createSparkFlex(
                RIGHT_MOTOR_ID, MotorType.kBrushless, false,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS),
                (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0));
    }

    public double getPosition(){
        return (Left.getEncoder().getPosition() + Right.getEncoder().getPosition()) / 2;
    }

    @Override
    public double getVelocity() {
        return (Left.getEncoder().getVelocity() + Right.getEncoder().getVelocity()) / 2;
    }

    @Override
    public void setVoltage(double voltage) {
        Left.setVoltage(voltage);
        Right.setVoltage(voltage);
    }

    @Override
    public Command spinAtVelocityCommand(Supplier<Double> goalVelocitySupplier) { //idk lol
        return run(() -> {
            feedbackVoltage = PIDController.calculate(getVelocity());
            feedforwardVoltage = feedforwardController.calculate(goalVelocitySupplier.get());
            setVoltage(feedbackVoltage + feedforwardVoltage);
        });
    }

    @Override
    public Command spinAtCurrentVelocityCommand() { //idk lol
        return runOnce(() -> spinAtVelocityCommand(() -> getVelocity()));

    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) { //idk
        return run(() -> setVoltage(12.0 * speed.get()))
                .withName("Set Overridden Shooter Speed");
    }

    @Override
    public Command coastMotorsCommand() { //idk
        return runOnce(() -> Left.stopMotor())
                .andThen(() -> {
                    Left.setIdleMode(IdleMode.kCoast);
                    Right.setIdleMode(IdleMode.kCoast);
                })
                .finallyDo((d) -> {
                    Left.setIdleMode(IdleMode.kBrake);
                    Right.setIdleMode(IdleMode.kBrake);
                    PIDController.reset();
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

}