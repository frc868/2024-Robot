package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.techhounds.houndutil.houndlib.PositionTracker;
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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.GlobalStates;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.NoteLift.Constants.*;

/**
 * The note lift subsystem, used to score in the trap. Handles motion profiling
 * and positioning of the carriage.
 * 
 * @apiNote This subsystem is no longer attached to the robot, and still exists
 *          for when the robot is in the pre-worlds configuration.
 */
@LoggedObject
public class NoteLift extends SubsystemBase implements BaseLinearMechanism<NoteLiftPosition> {
    public static final class Constants {
        public static enum NoteLiftPosition {
            BOTTOM(0),
            INTAKE(0.16),
            TRAVEL(0.4903106689453125),
            CLIMB_PREP(0.55),
            STOW(0.62),
            TOP(0.66);

            public final double value;

            private NoteLiftPosition(double value) {
                this.value = value;
            }
        }

        public static final int MOTOR_ID = 16;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNeoVortex(1);
        public static final double GEARING = 9;
        public static final boolean INVERTED = true;
        public static final double CARRIAGE_MASS_KG = Units.lbsToKilograms(3);
        public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(0.75);
        public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * DRUM_RADIUS_METERS;
        public static final double ENCODER_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / GEARING;

        public static final double MIN_HEIGHT_METERS = 0;
        public static final double MAX_HEIGHT_METERS = 0.69;

        public static final int CURRENT_LIMIT = 50;

        // 4/6/24
        public static final double kP = 45;
        public static final double kI = 0;
        public static final double kD = 1;
        public static final double kS = 0.28746;
        public static final double kG = -0.335364;
        public static final double kV = 8.142;
        public static final double kA = 0.386364;
        public static final double TOLERANCE = 0.01;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.7;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.7;
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final Pose3d BASE_COMPONENT_POSE = new Pose3d(0.177, 0, 0.20, new Rotation3d(0, 0, 0));
    }

    @Log
    private SparkFlex motor;
    private SparkFlexConfig motorConfig = new SparkFlexConfig();

    @Log(groups = "control")
    private ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, MOVEMENT_CONSTRAINTS);

    @Log(groups = "control")
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
            MAX_HEIGHT_METERS);

    @Log(groups = "control")
    private double feedbackVoltage = 0;
    @Log(groups = "control")
    private double feedforwardVoltage = 0;

    private double simVelocity = 0.0;

    private final MutVoltage sysidAppliedVoltageMeasure = Volts.mutable(0);
    private final MutDistance sysidPositionMeasure = Meters.mutable(0);
    private final MutLinearVelocity sysidVelocityMeasure = MetersPerSecond.mutable(0);

    private final SysIdRoutine sysIdRoutine;

    @Log
    private boolean initialized = false;

    private PositionTracker positionTracker;

    public NoteLift(PositionTracker positionTracker) {
        motor = new SparkFlex(MOTOR_ID, MotorType.kBrushless);
        
        motorConfig
            .inverted(INVERTED)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(CURRENT_LIMIT)
            .encoder
                .positionConversionFactor(ENCODER_ROTATIONS_TO_METERS)
                .velocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0);

        this.positionTracker = positionTracker;

        pidController.setTolerance(TOLERANCE);

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> setVoltage(volts.magnitude()),
                        log -> {
                            log.motor("primary")
                                    .voltage(sysidAppliedVoltageMeasure.mut_replace(motor.getAppliedOutput(), Volts))
                                    .linearPosition(sysidPositionMeasure.mut_replace(getPosition(), Meters))
                                    .linearVelocity(sysidVelocityMeasure.mut_replace(getVelocity(), MetersPerSecond));
                        },
                        this));

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
        motor.getEncoder().setPosition(NoteLiftPosition.TOP.value);
        initialized = true;
    }

    /**
     * Sets the voltage of the motors after applying limits.
     * 
     * <p>
     * Limits include: (1) soft stops between MIN_HEIGHT_METERS and
     * MAX_HEIGHT_METERS (with allowance for the mechanism to proceed 3cm beyond
     * this), (2) downwards movement lock-out if the climber is in the
     * path of the note lift, (3) downwards movement lock-out if the note lift is
     * above the shooter and is about to enter its path, (4) upwards movement
     * lock-out if the note lift is below the shooter and is about to enter its
     * path, and (5) total lock-out if the robot is not initialized.
     * 
     * @param voltage the voltage to set the motors to
     */
    @Override
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        if (!GlobalStates.MECH_LIMITS_DISABLED.enabled())
            voltage = Utils.applySoftStops(voltage, getPosition(), MIN_HEIGHT_METERS,
                    MAX_HEIGHT_METERS + 0.03); // allows note lift to unspool slightly

        if (!GlobalStates.INTER_SUBSYSTEM_SAFETIES_DISABLED.enabled()) {
            if (getPosition() - positionTracker.getPosition("climber") + 1 < -0.065 && voltage < 0) {
                voltage = 0;
            }
            if (getPosition() < 0.233 && positionTracker.getPosition("shooterTilt") < 1.11 && voltage > 0) {
                voltage = 0;
            }
            if (getPosition() > 0.233 && positionTracker.getPosition("shooterTilt") < 1.11 && voltage < 0) {
                voltage = 0;
            }
        }

        if (!GlobalStates.INITIALIZED.enabled() && !GlobalStates.INTER_SUBSYSTEM_SAFETIES_DISABLED.enabled()) {
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
                moveToCurrentGoalCommand().until(this::atGoal)).withName("noteLift.moveToPosition");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get())),
                moveToCurrentGoalCommand().until(this::atGoal)).withName("noteLift.moveToArbitraryPosition");
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
                .andThen(() -> {
                    motorConfig.idleMode(IdleMode.kCoast);
                    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                })
                .finallyDo((d) -> {
                    motorConfig.idleMode(IdleMode.kBrake);
                    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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

    /**
     * Creates a command that moves the note lift up to the top at maximum velocity,
     * until it slightly depools and triggers.
     * 
     * @return the command
     */
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

    public boolean getInitialized() {
        return initialized;
    }

    public Command setInitializedCommand(boolean initialized) {
        return Commands.runOnce(() -> {
            this.initialized = initialized;
        }).withName("noteLift.setInitialized");
    }

    /**
     * Creates a command that slowly moves the note lift up to the hard stop, and
     * initializes when a current spike is detected.
     * 
     * @apiNote currently unused
     * @return the command
     */
    public Command zeroMechanismCommand() {
        return run(() -> {
            motor.setVoltage(1);
        }).until(() -> motor.getOutputCurrent() > 10)
                .andThen(resetPositionCommand());
    }

    public boolean atGoal() {
        return pidController.atGoal() || GlobalStates.AT_GOAL_OVERRIDE.enabled();
    }
}
