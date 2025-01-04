package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.techhounds.houndutil.houndlib.SparkConfigurator;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlib.subsystems.BaseLinearMechanism;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Climber.ClimberPosition;
import frc.robot.GlobalStates;
import frc.robot.PositionTracker;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Climber.*;

/**
 * The climber subsystem, used to hang on the chain. Handles motion profiling
 * and positioning of the climbers.
 */
@LoggedObject
public class Climber extends SubsystemBase implements BaseLinearMechanism<ClimberPosition> {
    @Log
    private CANSparkFlex motor;

    @Log(groups = "control")
    private ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, MOVEMENT_CONSTRAINTS);

    private ElevatorFeedforward feedforwardController = new ElevatorFeedforward(kS, kG, kV, kA);

    /** The representation of the "elevator" for simulation. */
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

    /**
     * Updated the physics simulation, and sets data on motor controllers based on
     * its results.
     */
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

    /**
     * Gets the 3D relative pose of the climber based on its position, for use in
     * AdvantageScope.
     * 
     * @return the 3D pose of the climber
     */
    public Pose3d getComponentPose() {
        return BASE_COMPONENT_POSE.plus(new Transform3d(0, 0, getPosition(), new Rotation3d()));
    }

    @Override
    public void resetPosition() {
        motor.getEncoder().setPosition(ClimberPosition.BOTTOM.value);
        initialized = true;
    }

    /**
     * Sets the voltage of the motors after applying limits.
     * 
     * <p>
     * Limits include: (1) soft stops between MIN_HEIGHT_METERS and
     * MAX_HEIGHT_METERS, (2) downwards movement lock-out if the shooter is in the
     * path of the climber, (3) total lock-out if the climber's encoder value drops
     * below 0.5 (used at Worlds, to detect state if SPARK Flex erroneously rebooted
     * and reset position), and (4) total lock-out if the robot is not initialized.
     * 
     * @param voltage the voltage to set the motors to
     */
    @Override
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        if (!GlobalStates.MECH_LIMITS_DISABLED.enabled()) {
            voltage = Utils.applySoftStops(voltage, getPosition(), MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);
        }
        if (!GlobalStates.INTER_SUBSYSTEM_SAFETIES_DISABLED.enabled()) {
            if (getPosition() < 1.044 && positionTracker.getShooterTiltAngle() > 1.11 && voltage < 0) {
                voltage = 0;
            }
            if (getPosition() < 0.5) {
                voltage = 0;
            }
        }

        if (!GlobalStates.INITIALIZED.enabled() && !GlobalStates.INTER_SUBSYSTEM_SAFETIES_DISABLED.enabled()) {
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
                moveToCurrentGoalCommand().until(this::atGoal)).withName("climber.moveToPosition");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get())),
                moveToCurrentGoalCommand().until(this::atGoal)).withName("climber.moveToArbitraryPosition");
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

    /**
     * Creates a command that manually moves the climber down at maximum speed, and
     * stalls it against the end stop at 3V when it reaches the bottom, preventing
     * the robot from slowly moving up.
     * 
     * @return the command
     */
    public Command moveDownCommand() {
        return run(() -> {
            if (getPosition() > ClimberPosition.BOTTOM.value + 0.008) {
                setVoltage(-12);
            } else {
                setVoltage(-3);
            }
        })
                .finallyDo(() -> {
                    pidController.reset(getPosition());
                    pidController.setGoal(getPosition());
                });
    }

    /**
     * Creates a command that manually moves the climber up at maximum speed.
     * 
     * @return the command
     */
    public Command moveUpCommand() {
        return run(() -> setVoltage(12)).finallyDo(() -> {
            pidController.reset(getPosition());
            pidController.setGoal(getPosition());
        });
    }

    /**
     * Creates a command that resets the controller's goal to the current position
     * (used if overrides are enabled, then disabled, so that the mechanism does not
     * try to move to the previous goal before enabling overrides).
     * 
     * @return the command
     */
    public Command resetControllersCommand() {
        return Commands.runOnce(() -> pidController.reset(getPosition()))
                .andThen(Commands.runOnce(() -> pidController.setGoal(getPosition())));
    }

    @Log
    public boolean getInitialized() {
        return initialized;
    }

    public Command setInitializedCommand(boolean initialized) {
        return Commands.runOnce(() -> {
            this.initialized = initialized;
        }).withName("climber.setInitialized");
    }

    /**
     * Creates a command that slowly moves the climber down to the hard stop, and
     * initializes when a current spike is detected.
     * 
     * @apiNote currently unused
     * @return the command
     */
    public Command zeroMechanismCommand() {
        return run(() -> {
            motor.setVoltage(-1);
        }).until(new Trigger(() -> motor.getOutputCurrent() > 20).debounce(1))
                .andThen(resetPositionCommand());
    }

    public boolean atGoal() {
        return pidController.atGoal() || GlobalStates.AT_GOAL_OVERRIDE.enabled();
    }
}
