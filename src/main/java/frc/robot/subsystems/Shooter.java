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

import edu.wpi.first.math.MathUtil;
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
    private final CANSparkFlex leftMotor;

    @Log
    private final CANSparkFlex rightMotor;

    @Log(groups = "control")
    private final PIDController leftPidController = new PIDController(kP, kI, kD);
    @Log(groups = "control")
    private final PIDController rightPidController = new PIDController(kP, kI, kD);

    @Log(groups = "control")
    private final SimpleMotorFeedforward feedforwardController = new SimpleMotorFeedforward(kS, kV, kA);

    private final FlywheelSim flywheelSim = new FlywheelSim(MOTOR_GEARBOX_REPR, GEARING,
            MOMENT_OF_INERTIA_KG_METERS_SQUARED);

    @Log(groups = "control")
    private double feedforwardVoltage = 0.0;
    @Log(groups = "control")
    private double leftFeedbackVoltage = 0.0;
    @Log(groups = "control")
    private double rightFeedbackVoltage = 0.0;

    private double simVelocity = 0.0;

    private final MutableMeasure<Voltage> sysidAppliedVoltageMeasure = MutableMeasure.mutable(Volts.of(0));
    private final MutableMeasure<Angle> sysidPositionMeasure = MutableMeasure.mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> sysidVelocityMeasure = MutableMeasure
            .mutable(RotationsPerSecond.of(0));

    private final SysIdRoutine sysIdRoutine;

    public Shooter() {
        leftMotor = SparkConfigurator.createSparkFlex(LEFT_MOTOR_ID, MotorType.kBrushless, true,
                (s) -> s.setIdleMode(IdleMode.kCoast),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(1.0),
                (s) -> s.getEncoder().setVelocityConversionFactor(1.0 / 60.0));

        rightMotor = SparkConfigurator.createSparkFlex(RIGHT_MOTOR_ID, MotorType.kBrushless, true,
                (s) -> s.setIdleMode(IdleMode.kCoast),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(1.0),
                (s) -> s.getEncoder().setVelocityConversionFactor(1.0 / 60.0));

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> setVoltage(volts.magnitude()),
                        log -> {
                            log.motor("left")
                                    .voltage(sysidAppliedVoltageMeasure.mut_replace(leftMotor.getAppliedOutput(),
                                            Volts))
                                    .angularPosition(sysidPositionMeasure.mut_replace(0, Rotations))
                                    .angularVelocity(
                                            sysidVelocityMeasure.mut_replace(leftMotor.getEncoder().getVelocity(),
                                                    RotationsPerSecond));
                            log.motor("right")
                                    .voltage(sysidAppliedVoltageMeasure.mut_replace(rightMotor.getAppliedOutput(),
                                            Volts))
                                    .angularPosition(sysidPositionMeasure.mut_replace(0, Rotations))
                                    .angularVelocity(
                                            sysidVelocityMeasure.mut_replace(rightMotor.getEncoder().getVelocity(),
                                                    RotationsPerSecond));
                        },
                        this));

        setDefaultCommand(holdVelocityCommand(() -> IDLE_RPS));
        leftPidController.setTolerance(TOLERANCE);
        rightPidController.setTolerance(TOLERANCE);

    }

    @Override
    public void simulationPeriodic() {
        // set the input (the voltage of the motor)
        flywheelSim.setInput(leftMotor.getAppliedOutput());
        // update the sim
        flywheelSim.update(0.020);
        simVelocity = flywheelSim.getAngularVelocityRPM() / 60.0;
    }

    @Override
    public double getVelocity() {
        if (RobotBase.isReal())
            return leftMotor.getEncoder().getVelocity();
        else
            return simVelocity;
    }

    @Log
    public double getLeftVelocity() {
        if (RobotBase.isReal())
            return leftMotor.getEncoder().getVelocity();
        else
            return simVelocity;
    }

    @Log
    public double getRightVelocity() {
        if (RobotBase.isReal())
            return rightMotor.getEncoder().getVelocity();
        else
            return simVelocity;
    }

    @Override
    public void setVoltage(double voltage) {
        leftMotor.setVoltage(MathUtil.clamp(voltage, -12, 12));
        rightMotor.setVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    public void setLeftVoltage(double voltage) {
        leftMotor.setVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    public void setRightVoltage(double voltage) {
        rightMotor.setVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    public void stop() {
        leftMotor.setVoltage(0);
        rightMotor.setVoltage(0);
    }

    @Override
    public Command spinAtVelocityCommand(Supplier<Double> goalVelocitySupplier) {
        return run(() -> {
            leftFeedbackVoltage = leftPidController.calculate(getLeftVelocity(), goalVelocitySupplier.get());
            rightFeedbackVoltage = rightPidController.calculate(getRightVelocity(), goalVelocitySupplier.get());
            feedforwardVoltage = feedforwardController.calculate(goalVelocitySupplier.get());
            setLeftVoltage(leftFeedbackVoltage + feedforwardVoltage);
            setRightVoltage(rightFeedbackVoltage + feedforwardVoltage);
        }).withName("shooter.spinAtVelocity");
    }

    // will get up to speed, but slowly ramp down at only FF
    public Command holdVelocityCommand(Supplier<Double> goalVelocitySupplier) {
        return run(() -> {
            leftFeedbackVoltage = leftPidController.calculate(getLeftVelocity(), goalVelocitySupplier.get());
            rightFeedbackVoltage = rightPidController.calculate(getRightVelocity(), goalVelocitySupplier.get());
            feedforwardVoltage = feedforwardController.calculate(goalVelocitySupplier.get());
            if (leftFeedbackVoltage < 0)
                leftFeedbackVoltage = 0.0;
            if (rightFeedbackVoltage < 0)
                rightFeedbackVoltage = 0.0;
            setLeftVoltage(leftFeedbackVoltage + feedforwardVoltage);
            setRightVoltage(rightFeedbackVoltage + feedforwardVoltage);
        }).withName("shooter.holdVelocity");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
                .withName("shooter.setOverriddenSpeed");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> leftMotor.stopMotor())
                .andThen(() -> {
                    leftMotor.setIdleMode(IdleMode.kCoast);
                    rightMotor.setIdleMode(IdleMode.kCoast);
                })
                .finallyDo((d) -> {
                    leftMotor.setIdleMode(IdleMode.kBrake);
                    rightMotor.setIdleMode(IdleMode.kBrake);
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("shooter.coastMotors");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction).withName("shooter.sysIdQuasistatic");
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction).withName("shooter.sysIdDynamic");
    }

    @Log
    public boolean atGoal() {
        return leftPidController.atSetpoint() && rightPidController.atSetpoint();
    }
}
