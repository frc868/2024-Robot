package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.techhounds.houndutil.houndlib.SparkConfigurator;
import com.techhounds.houndutil.houndlib.subsystems.BaseShooter;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Shooter.*;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@LoggedObject
public class Shooter extends SubsystemBase implements BaseShooter {
    @Log
    private final CANSparkFlex primaryMotor;

    @Log
    private final CANSparkFlex secondaryMotor;

    @Log
    private final PIDController pidController = new PIDController(kP, kI, kD);
    @Log
    private final SimpleMotorFeedforward feedforwardController = new SimpleMotorFeedforward(kS, kV, kA);

    private final FlywheelSim flywheelSim = new FlywheelSim(MOTOR_GEARBOX_REPR, GEARING,
            MOMENT_OF_INERTIA_KG_METERS_SQUARED);

    @Log
    private double feedforwardVoltage = 0.0;
    @Log
    private double feedbackVoltage = 0.0;

    private double simVelocity = 0.0;

    private final MutableMeasure<Voltage> sysidAppliedVoltageMeasure = MutableMeasure.mutable(Volts.of(0));
    private final MutableMeasure<Angle> sysidPositionMeasure = MutableMeasure.mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> sysidVelocityMeasure = MutableMeasure
            .mutable(RotationsPerSecond.of(0));

    private final SysIdRoutine sysIdRoutine;

    public Shooter() {
        primaryMotor = SparkConfigurator.createSparkFlex(PRIMARY_MOTOR_ID, MotorType.kBrushless, false,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(1.0),
                (s) -> s.getEncoder().setVelocityConversionFactor(1.0 / 60.0));

        secondaryMotor = SparkConfigurator.createSparkFlex(SECONDARY_MOTOR_ID, MotorType.kBrushless, false,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.follow(primaryMotor, true));

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> setVoltage(volts.magnitude()),
                        log -> {
                            log.motor("primary")
                                    .voltage(sysidAppliedVoltageMeasure.mut_replace(primaryMotor.getAppliedOutput(),
                                            Volts))
                                    .angularPosition(sysidPositionMeasure.mut_replace(0, Rotations))
                                    .angularVelocity(
                                            sysidVelocityMeasure.mut_replace(getVelocity(), RotationsPerSecond));
                        },
                        this));

        setDefaultCommand(holdVelocityCommand(() -> IDLE_RPS));
        pidController.setTolerance(0.25);

    }

    @Override
    public void simulationPeriodic() {
        // set the input (the voltage of the motor)
        flywheelSim.setInput(primaryMotor.getAppliedOutput());
        // update the sim
        flywheelSim.update(0.020);
        simVelocity = flywheelSim.getAngularVelocityRPM() / 60.0;
    }

    @Override
    public double getVelocity() {
        if (RobotBase.isReal())
            return primaryMotor.getEncoder().getVelocity();
        else
            return simVelocity;
    }

    @Override
    public void setVoltage(double voltage) {
        primaryMotor.setVoltage(voltage > 12 ? 12 : (voltage < -12 ? -12 : voltage));
    }

    public void stop() {
        primaryMotor.setVoltage(0);
    }

    @Override
    public Command spinAtVelocityCommand(Supplier<Double> goalVelocitySupplier) {
        return run(() -> {
            feedbackVoltage = pidController.calculate(getVelocity(), goalVelocitySupplier.get());
            feedforwardVoltage = feedforwardController.calculate(goalVelocitySupplier.get());
            setVoltage(feedbackVoltage + feedforwardVoltage);
        }).withName("shooter.spinAtVelocity");
    }

    // will get up to speed, but slowly ramp down
    public Command holdVelocityCommand(Supplier<Double> goalVelocitySupplier) {
        return run(() -> {
            feedbackVoltage = pidController.calculate(getVelocity(), goalVelocitySupplier.get());
            feedforwardVoltage = feedforwardController.calculate(goalVelocitySupplier.get());
            if (feedbackVoltage < 0)
                feedbackVoltage = 0.0;
            setVoltage(feedbackVoltage + feedforwardVoltage);
        }).withName("shooter.holdVelocity");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return run(() -> setVoltage(12.0 * speed.get()))
                .withName("shooter.setOverriddenSpeed");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> primaryMotor.stopMotor())
                .andThen(() -> {
                    primaryMotor.setIdleMode(IdleMode.kCoast);
                    secondaryMotor.setIdleMode(IdleMode.kCoast);
                })
                .finallyDo((d) -> {
                    primaryMotor.setIdleMode(IdleMode.kBrake);
                    secondaryMotor.setIdleMode(IdleMode.kBrake);
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("shooter.coastMotors");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction).withName("shooter.sysIdQuasistatic");
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction).withName("shooter.sysIdDynamic");
    }

    public boolean atGoal() {
        return pidController.atSetpoint();
    }
}
