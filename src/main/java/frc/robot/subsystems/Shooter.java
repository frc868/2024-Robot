package frc.robot.subsystems;

import static frc.robot.Constants.Shooter.*;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.techhounds.houndutil.houndlib.SparkConfigurator;
import com.techhounds.houndutil.houndlib.subsystems.BaseShooter;
import com.techhounds.houndutil.houndlog.interfaces.Log;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase implements BaseShooter {

    // example motor
    private CANSparkFlex primaryMotor;
    private CANSparkFlex secondaryMotor;
    // example velocity variable

    private SimpleMotorFeedforward feedforwardController = new SimpleMotorFeedforward(kS, kV, kA);
    private PIDController pidController = new PIDController(kP, kI, kD);

    @Log(groups = "control")
    private double feedbackVoltage = 0;
    @Log(groups = "control")
    private double feedforwardVoltage = 0;

    public Shooter() {
        primaryMotor = SparkConfigurator.createSparkFlex(
                PRIMARY_MOTOR_ID, MotorType.kBrushless, false,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS),
                (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0));
        secondaryMotor = SparkConfigurator.createSparkFlex(
                SECONDARY_MOTOR_ID, MotorType.kBrushless, false,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS),
                (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0));

        secondaryMotor.follow(primaryMotor);
    }

    @Override
    public double getVelocity() {
        return (primaryMotor.getEncoder().getVelocity());
    }

    @Override
    public void setVoltage(double voltage) {
        primaryMotor.setVoltage(voltage);
        secondaryMotor.setVoltage(voltage);
    }

    @Override
    public Command spinAtVelocityCommand(Supplier<Double> goalVelocitySupplier) { // idk lol
        return run(() -> {
            feedbackVoltage = pidController.calculate(getVelocity(), goalVelocitySupplier.get()); // ?
            feedforwardVoltage = feedforwardController.calculate(goalVelocitySupplier.get());
            setVoltage(feedbackVoltage + feedforwardVoltage);
        });
    }

    public Command idleAtVoltageCommand(double voltage) {
        return run(() -> {
            feedbackVoltage = getVelocity() < pidController.getSetpoint()
                    ? pidController.calculate(pidController.getSetpoint())
                    : 0; // ?
            setVoltage(feedbackVoltage + voltage);
        });
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) { // idk
        return run(() -> setVoltage(12.0 * speed.get()))
                .withName("Set Overridden Shooter Speed");
    }

    @Override
    public Command coastMotorsCommand() { // idk
        return runOnce(() -> primaryMotor.stopMotor())
                .andThen(() -> {
                    primaryMotor.setIdleMode(IdleMode.kCoast);
                    secondaryMotor.setIdleMode(IdleMode.kCoast);
                })
                .finallyDo((d) -> {
                    primaryMotor.setIdleMode(IdleMode.kBrake);
                    secondaryMotor.setIdleMode(IdleMode.kBrake);
                    pidController.reset();
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

}
