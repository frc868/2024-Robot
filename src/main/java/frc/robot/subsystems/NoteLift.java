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
import frc.robot.Constants.NoteLift.NoteLiftPosition;
import frc.robot.GlobalStates;
import frc.robot.PositionTracker;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.NoteLift.*;

@LoggedObject
public class NoteLift extends SubsystemBase implements BaseElevator<NoteLiftPosition> {
    @Log
    private CANSparkFlex motor;

    @Log(groups = "control")
    private ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, MOVEMENT_CONSTRAINTS);

    @Log(groups = "control")
    private ElevatorFeedforward feedforwardController = new ElevatorFeedforward(kS, kG, kV, kA);

    private ElevatorSim elevatorSim = new ElevatorSim(
            MOTOR_GEARBOX_REPR,
            GEARING,
            CARRIAGE_MASS_KG,
            DRUM_RADIUS_METERS,
            MIN_HEIGHT_METERS,
            MAX_HEIGHT_METERS,
            false,
            MAX_HEIGHT_METERS);

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

    @Log
    private boolean initialized = false;

    private PositionTracker positionTracker;

    public NoteLift(PositionTracker positionTracker) {
        motor = SparkConfigurator.createSparkFlex(
                MOTOR_ID, MotorType.kBrushless, true,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS),
                (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0));

        this.positionTracker = positionTracker;

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
        motor.getEncoder().setPosition(NoteLiftPosition.TOP.value);
        initialized = true;
    }

    @Override
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        voltage = Utils.applySoftStops(voltage, getPosition(), MIN_HEIGHT_METERS,
                MAX_HEIGHT_METERS + 0.03); // allows note lift to unspool slightly

        if (getPosition() - positionTracker.getClimberPosition() < -0.09 && voltage < 0) {
            voltage = 0;
        }
        if (getPosition() < 0.233 && positionTracker.getShooterTiltAngle() < 1.11 && voltage > 0) {
            voltage = 0;
        }
        if (getPosition() > 0.233 && positionTracker.getShooterTiltAngle() < 1.11 && voltage < 0) {
            voltage = 0;
        }

        if (!GlobalStates.INITIALIZED.enabled()) {
            voltage = 0.0;
        }
        motor.setVoltage(voltage);
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            feedbackVoltage = pidController.calculate(getPosition());
            feedforwardVoltage = feedforwardController.calculate(pidController.getSetpoint().velocity);
            setVoltage(feedbackVoltage + feedforwardVoltage);
        }).withName("noteLift.moveToCurrentGoal");
    }

    @Override
    public Command moveToPositionCommand(Supplier<NoteLiftPosition> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get().value)),
                moveToCurrentGoalCommand().until(pidController::atGoal)).withName("noteLift.moveToPosition");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get())),
                moveToCurrentGoalCommand().until(pidController::atGoal)).withName("noteLift.moveToArbitraryPosition");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> pidController.getGoal().position + delta.get())
                .withName("noteLift.movePositionDelta");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> pidController.setGoal(getPosition())).andThen(moveToCurrentGoalCommand())
                .withName("noteLift.holdCurrentPosition");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("noteLift.resetPosition");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
                .withName("noteLift.setOverriddenSpeed");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(motor::stopMotor)
                .andThen(() -> motor.setIdleMode(IdleMode.kCoast))
                .finallyDo((d) -> {
                    motor.setIdleMode(IdleMode.kBrake);
                    pidController.reset(getPosition());
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("noteLift.coastMotorsCommand");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction).withName("noteLift.sysIdQuasistatic");
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction).withName("noteLift.sysIdDynamic");
    }

    public Command scoreNoteCommand() {
        return run(() -> {
            if (getPosition() < MAX_HEIGHT_METERS + 0.03)
                setVoltage(12);
            else
                setVoltage(0);
        }).finallyDo(() -> {
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

    public Command enableInitializedCommand() {
        return Commands.runOnce(() -> {
            initialized = true;
        }).withName("shooterTilt.enableInitialized");
    }

    public Command zeroMechanismCommand() {
        return run(() -> {
            motor.setVoltage(1);
        }).until(() -> motor.getOutputCurrent() > 10)
                .andThen(resetPositionCommand());
    }
}
