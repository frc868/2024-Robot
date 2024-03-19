package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.techhounds.houndutil.houndlib.SparkConfigurator;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlib.subsystems.BaseElevator;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.Climber.ClimberPosition;
import frc.robot.GlobalStates;
import frc.robot.PositionTracker;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Climber.*;

@LoggedObject
public class Climber extends SubsystemBase implements BaseElevator<ClimberPosition> {
    @Log
    private CANSparkFlex motor;

    @Log(groups = "control")
    private ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, MOVEMENT_CONSTRAINTS);

    private ElevatorFeedforward feedforwardController = new ElevatorFeedforward(kS, kG, kV, kA);

    private ElevatorSim elevatorSim = new ElevatorSim(
            MOTOR_GEARBOX_REPR,
            GEARING,
            CARRIAGE_MASS_KG,
            DRUM_RADIUS_METERS,
            MIN_HEIGHT_METERS,
            MAX_HEIGHT_METERS,
            false,
            0);

    @Log(groups = "control")
    private double feedbackVoltage = 0;
    @Log(groups = "control")
    private double feedforwardVoltage = 0;

    private double simVelocity = 0.0;

    private final MutableMeasure<Voltage> sysidAppliedVoltageMeasure = MutableMeasure.mutable(Volts.of(0));
    private final MutableMeasure<Distance> sysidPositionMeasure = MutableMeasure.mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> sysidVelocityMeasure = MutableMeasure
            .mutable(MetersPerSecond.of(0));

    private final SysIdRoutine sysIdRoutine;

    private boolean initialized = false;

    private PositionTracker positionTracker;

    public Climber(PositionTracker positionTracker) {
        motor = SparkConfigurator.createSparkFlex(
                MOTOR_ID, MotorType.kBrushless, true,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS),
                (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0));

        pidController.setTolerance(TOLERANCE);

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> setVoltage(volts.magnitude()),
                        log -> {
                            log.motor("primary")
                                    .voltage(sysidAppliedVoltageMeasure.mut_replace(motor.getAppliedOutput(), Volts))
                                    .linearPosition(sysidPositionMeasure.mut_replace(getPosition(), Meters))
                                    .linearVelocity(sysidVelocityMeasure.mut_replace(getVelocity(), MetersPerSecond));
                        },
                        this));

        this.positionTracker = positionTracker;

        setDefaultCommand(moveToCurrentGoalCommand());
    }

    @Override
    public void simulationPeriodic() {
        elevatorSim.setInput(motor.getAppliedOutput());
        elevatorSim.update(0.020);
        motor.getEncoder().setPosition(elevatorSim.getPositionMeters());
        simVelocity = elevatorSim.getVelocityMetersPerSecond();
    }

    @Override
    @Log
    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    @Log
    public double getVelocity() {
        if (RobotBase.isReal())
            return motor.getEncoder().getVelocity();
        else
            return simVelocity;
    }

    public Pose3d getComponentPose() {
        return BASE_COMPONENT_POSE.plus(new Transform3d(0, 0, getPosition(), new Rotation3d()));
    }

    @Override
    public void resetPosition() {
        motor.getEncoder().setPosition(0);
        initialized = true;
    }

    @Override
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        if (!GlobalStates.MECH_LIMITS_DISABLED.enabled()) {
            voltage = Utils.applySoftStops(voltage, getPosition(), MIN_HEIGHT_METERS - 0.05, MAX_HEIGHT_METERS);
            if (getPosition() < 0.02) {
                voltage = MathUtil.clamp(voltage, -3, 12);
            }
        }
        if (!GlobalStates.INTER_SUBSYSTEM_SAFETIES_DISABLED.enabled()) {
            if (getPosition() < 0.293 && positionTracker.getShooterTiltAngle() < 1.11 && voltage > 0) {
                voltage = 0;
            }
            if (getPosition() > 0.293 && positionTracker.getShooterTiltAngle() < 1.11 && voltage < 0) {
                voltage = 0;
            }

            if (positionTracker.getNoteLiftPosition() - getPosition() < -0.09 && voltage > 0) {
                voltage = 0;
            }
        }

        if (!GlobalStates.INITIALIZED.enabled()) {
            voltage = 0;
        }
        motor.setVoltage(voltage);
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            feedbackVoltage = pidController.calculate(getPosition());
            feedforwardVoltage = feedforwardController.calculate(pidController.getSetpoint().velocity);
            setVoltage(feedbackVoltage + feedforwardVoltage);
        }).withName("climber.moveToCurrentGoal");
    }

    @Override
    public Command moveToPositionCommand(Supplier<ClimberPosition> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get().value)),
                moveToCurrentGoalCommand().until(pidController::atGoal)).withName("climber.moveToPosition");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get())),
                moveToCurrentGoalCommand().until(pidController::atGoal)).withName("climber.moveToArbitraryPosition");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> pidController.getGoal().position + delta.get())
                .withName("climber.movePositionDelta");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> pidController.setGoal(getPosition())).andThen(moveToCurrentGoalCommand())
                .withName("climber.holdCurrentPosition");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("climber.resetPosition");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
                .withName("climber.setOverriddenSpeed");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(motor::stopMotor)
                .andThen(() -> motor.setIdleMode(IdleMode.kCoast))
                .finallyDo((d) -> {
                    motor.setIdleMode(IdleMode.kBrake);
                    pidController.reset(getPosition());
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("climber.coastMotorsCommand");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction).withName("climber.sysIdQuasistatic");
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction).withName("climber.sysIdDynamic");
    }

    public Command climbToBottomCommand() {
        return run(() -> {
            if (getPosition() > ClimberPosition.BOTTOM.value + 0.008) {
                setVoltage(-9);
            } else {
                setVoltage(-3);
            }
        })
                .until(() -> getPosition() < ClimberPosition.BOTTOM.value + 0.008)
                .finallyDo(() -> {
                    pidController.reset(getPosition());
                    pidController.setGoal(getPosition());
                });
    }

    public Command resetControllersCommand() {
        return Commands.runOnce(() -> pidController.reset(getPosition()))
                .andThen(Commands.runOnce(() -> pidController.setGoal(getPosition())));
    }

    public boolean getInitialized() {
        return initialized;
    }

    public Command setInitializedCommand(boolean initialized) {
        return Commands.runOnce(() -> {
            this.initialized = initialized;
        }).withName("climber.setInitialized");
    }

    public Command zeroMechanismCommand() {
        return run(() -> {
            motor.setVoltage(-1);
        }).until(() -> motor.getOutputCurrent() > 10)
                .andThen(resetPositionCommand());
    }

    public boolean atGoal() {
        return pidController.atGoal() || GlobalStates.AT_GOAL_OVERRIDE.enabled();
    }
}
