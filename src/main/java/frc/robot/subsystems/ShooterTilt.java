package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.techhounds.houndutil.houndauto.Reflector;
import com.techhounds.houndutil.houndlib.PositionTracker;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;
import com.techhounds.houndutil.houndlog.loggers.TunableDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants;
import frc.robot.GlobalStates;
import static frc.robot.subsystems.Shooter.Constants.MAX_SHOOTING_DISTANCE;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.ShooterTilt.Constants.*;
/**
 * The shooter tilt subsystem, used to control the angle of the shooter via the
 * attached lead screw. Handles motion profiling and positioning of the screw.
 */
@LoggedObject
public class ShooterTilt extends SubsystemBase implements BaseSingleJointedArm<ShooterTiltPosition> {
    public static final class Constants {
        // 0.160 max movement
        // 2/15/2024
        public static enum ShooterTiltPosition {
            BOTTOM(0.389842),
            AMP_EJECT(0.601),
            INTAKE(0.585),
            CLIMB(1.176781),
            SUBWOOFER(1.01572),
            PASS(0.90),
            PODIUM(0.651760);

            public final double value;

            private ShooterTiltPosition(double value) {
                this.value = value;
            }
        }

        public static final int MOTOR_ID = 14;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNeoVortex(1);
        public static final double GEARING = 5.0;
        public static final double MASS_KG = Units.lbsToKilograms(12);
        // 1 rot = 12mm
        public static final double ENCODER_ROTATIONS_TO_METERS = Units.inchesToMeters(0.5) / GEARING;

        public static final double MIN_HEIGHT_METERS = 0.005;
        public static final double MAX_HEIGHT_METERS = 0.14;

        public static final double MIN_ANGLE_RADIANS = 0.389842;
        public static final double MAX_ANGLE_RADIANS = 1.2;
        public static final TunableDouble DEMO_ANGLE = new TunableDouble("subsystems/shooterTilt/DEMO_ANGLE",
                1.01572);

        public static final int CURRENT_LIMIT = 25;

        // 3/28/24
        public static final double kP = 400;
        public static final double kI = 0;
        public static final double kD = 2;
        public static final double kS = 0.104904;
        public static final double kG = 0.124356;
        public static final double kV = 44.0148;
        public static final double kA = 3.48876;
        public static final double TOLERANCE = 0.02;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.3;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.32;
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final Pose3d BASE_SHOOTER_POSE = new Pose3d(-0.19, 0, 0.299, new Rotation3d(0, 0, 0));
        public static final Pose3d BASE_OUTER_LEAD_SCREW_POSE = new Pose3d(-0.0915,
                0, 0.125,
                new Rotation3d(0, 0, 0));

        public static final Transform3d OUTER_LEAD_SCREW_TO_INNER_LEAD_SCREW = new Transform3d(0.047, 0, 0.0127,
                new Rotation3d());
        public static final Transform3d LEAD_SCREW_PIVOT_TO_EXTENSION = new Transform3d(0, 0, 0.0127,
                new Rotation3d());
        public static final Transform3d SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT = new Transform3d(0.2572, 0,
                -0.0984,
                new Rotation3d());
        public static final double INITIAL_LEAD_SCREW_LENGTH = 0.2519;

        public static final Transform3d SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT = new Transform3d(0.09906, 0,
                -0.1737,
                new Rotation3d());

        /**
         * Gets the length of the lead screw given the angle of the shooter tilt.
         * 
         * @param angle the angle of the shooter tilt
         * @return the length of the lead screw
         */
        public static final double getLinearActuatorLength(double angle) {
            double shooterToBottomLeadScrewAngle = Math
                    .atan(Math.abs(SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getX())
                            / Math.abs(SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getZ()));
            double shooterToTopLeadScrewAngle = Math
                    .atan(Math.abs(SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT.getZ())
                            / Math.abs(SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT.getX()));

            double shooterInteriorAngle = angle - shooterToBottomLeadScrewAngle - shooterToTopLeadScrewAngle
                    + Math.PI / 2.0;

            double shooterPivotToBottomPivot = SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getTranslation()
                    .getNorm(); // a
            double shooterPivotToTopPivot = SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT.getTranslation()
                    .getNorm(); // b

            return Math.sqrt(
                    Math.pow(shooterPivotToBottomPivot, 2)
                            - (2 * shooterPivotToBottomPivot * shooterPivotToTopPivot
                                    * Math.cos(shooterInteriorAngle))
                            + Math.pow(shooterPivotToTopPivot, 2)
                            - Math.pow(LEAD_SCREW_PIVOT_TO_EXTENSION.getZ(), 2))
                    - INITIAL_LEAD_SCREW_LENGTH;
        }

        /**
         * Gets the angle of the shooter tilt given the length of the lead screw.
         * 
         * <p>
         * https://www.desmos.com/calculator/7ojeknrpiq
         * 
         * @param linearActuatorLength the length of the lead screw
         * @return the angle of the shooter tilt
         */
        public static final double getShooterAngle(double linearActuatorLength) {
            double fullLeadScrewLength = INITIAL_LEAD_SCREW_LENGTH + linearActuatorLength;

            double shooterPivotToBottomPivot = SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getTranslation()
                    .getNorm(); // a
            double shooterPivotToTopPivot = SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT.getTranslation()
                    .getNorm(); // b
            double leadScrewHyp = Math
                    .sqrt(Math.pow(LEAD_SCREW_PIVOT_TO_EXTENSION.getZ(), 2)
                            + Math.pow(fullLeadScrewLength, 2)); // c

            double shooterInteriorAngle = Math.acos(
                    (Math.pow(shooterPivotToBottomPivot, 2)
                            + Math.pow(shooterPivotToTopPivot, 2)
                            - Math.pow(leadScrewHyp, 2))
                            / (2 * shooterPivotToBottomPivot * shooterPivotToTopPivot));

            double shooterToBottomLeadScrewAngle = Math
                    .atan(Math.abs(SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getX())
                            / Math.abs(SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getZ()));
            double shooterToTopLeadScrewAngle = Math
                    .atan(Math.abs(SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT.getZ())
                            / Math.abs(SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT.getX()));

            return shooterInteriorAngle + shooterToBottomLeadScrewAngle + shooterToTopLeadScrewAngle
                    - Math.PI / 2.0;
        }

        /**
         * Gets the angle of the lead screw from its rotation axis given its length.
         * Used for simulation.
         * 
         * @param linearActuatorLength the length of the lead screw
         * @return the angle of the lead screw
         */
        public static final double getLeadScrewAngle(double linearActuatorLength) {
            double fullLeadScrewLength = INITIAL_LEAD_SCREW_LENGTH + linearActuatorLength;

            double shooterPivotToBottomPivot = SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getTranslation()
                    .getNorm(); // a
            double shooterPivotToTopPivot = SHOOTER_PIVOT_TO_TOP_LEAD_SCREW_PIVOT.getTranslation()
                    .getNorm(); // b
            double leadScrewHyp = Math
                    .sqrt(Math.pow(LEAD_SCREW_PIVOT_TO_EXTENSION.getZ(), 2)
                            + Math.pow(fullLeadScrewLength, 2)); // c

            double leadScrewInteriorAngle = Math.acos(
                    (Math.pow(shooterPivotToBottomPivot, 2)
                            + Math.pow(leadScrewHyp, 2)
                            - Math.pow(shooterPivotToTopPivot, 2))
                            / (2 * shooterPivotToBottomPivot * leadScrewHyp));

            double bottomLeadScrewToShooterAngle = Math
                    .atan(Math.abs(SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getZ())
                            / Math.abs(SHOOTER_PIVOT_TO_BOTTOM_LEAD_SCREW_PIVOT.getX()));

            return Math.PI - leadScrewInteriorAngle - bottomLeadScrewToShooterAngle;
        }

        /**
         * Interpolator tht takes in the xy distance from the target and returns the
         * setpoint tilt angle.
         */
        public static final InterpolatingDoubleTreeMap LEAD_SCREW_HEIGHT_INTERPOLATOR = new InterpolatingDoubleTreeMap();
        static {
            // 3/5/24
            LEAD_SCREW_HEIGHT_INTERPOLATOR.put(1.1795122686667598, 0.10887146);
            LEAD_SCREW_HEIGHT_INTERPOLATOR.put(1.7062360399365777, 0.08586998291);
            LEAD_SCREW_HEIGHT_INTERPOLATOR.put(2.235756419155122, 0.06745673828);
            LEAD_SCREW_HEIGHT_INTERPOLATOR.put(2.9124685062916904, 0.04472818315);
            LEAD_SCREW_HEIGHT_INTERPOLATOR.put(3.419734229712845, 0.03299551314);
            LEAD_SCREW_HEIGHT_INTERPOLATOR.put(3.948809911586509, 0.02273276727);
            LEAD_SCREW_HEIGHT_INTERPOLATOR.put(4.239, 0.01725753836);
            LEAD_SCREW_HEIGHT_INTERPOLATOR.put(4.597, 0.0133918667);
            LEAD_SCREW_HEIGHT_INTERPOLATOR.put(5.1322, 0.009597811761);

        }
    
    }
    @Log
    private final SparkFlex motor;
    private SparkFlexConfig motorConfig = new SparkFlexConfig();
    @Log(groups = "control")
    private final ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, MOVEMENT_CONSTRAINTS);

    @Log(groups = "control")
    private final ElevatorFeedforward feedforwardController = new ElevatorFeedforward(kS,
            kG, kV, kA);

    /**
     * The representation of the "elevator" for simulation. (even though this is a
     * rotational mechanism w.r.t. its setpoints, we still control it as a linear
     * mechanism since that is the cloest physical mechanism to this)
     */
    private final ElevatorSim elevatorSim = new ElevatorSim(
            MOTOR_GEARBOX_REPR,
            1 / ENCODER_ROTATIONS_TO_METERS, // includes lead screw pitch
            MASS_KG,
            1 / (2.0 * Math.PI), // negates the effect of the spool radius (no spool on a lead screw)
            MIN_HEIGHT_METERS,
            MAX_HEIGHT_METERS,
            false,
            ShooterTiltPosition.SUBWOOFER.value);

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
    private double distance = 0.0;
    @Log
    private boolean beyondMaxDistance = false;
    @Log
    private boolean initialized = false;
    @SuppressWarnings("unused")
    private PositionTracker positionTracker;

    public ShooterTilt(PositionTracker positionTracker) {
        motor = new SparkFlex(MOTOR_ID, MotorType.kBrushless);
        motorConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(CURRENT_LIMIT)
            .encoder
                .positionConversionFactor(ENCODER_ROTATIONS_TO_METERS)
                .velocityConversionFactor(ENCODER_ROTATIONS_TO_METERS/60.0);
        motor.configure(motorConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

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

        this.positionTracker = positionTracker;

        pidController.setTolerance(TOLERANCE);
        pidController.setGoal(getLinearActuatorLength(ShooterTiltPosition.BOTTOM.value));

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                pidController.reset(getPosition());
            } catch (Exception e) {
                e.printStackTrace(System.err);
            }
        }).start();

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

    /**
     * Gets the 3D relative pose of the shooter based on its position, for use in
     * AdvantageScope.
     * 
     * @return the 3D pose of the shooter
     */
    public Pose3d getShooterComponentPose() {
        return BASE_SHOOTER_POSE.plus(new Transform3d(0, 0, 0, new Rotation3d(0, -getAngle(), 0)));
    }

    /**
     * Gets the 3D relative pose of the outer component of the lead screw based on
     * its position, for use in AdvantageScope.
     * 
     * @return the 3D pose of the outer component of the lead screw
     */
    public Pose3d getOuterLeadScrewComponentPose() {
        // small offset of 0.03rad added to fix an alignment issue between the two;
        // since this is constant, it's likely a CAD zeroing problem
        return BASE_OUTER_LEAD_SCREW_POSE
                .plus(new Transform3d(0, 0, 0, new Rotation3d(0, -getLeadScrewAngle(getPosition()) + 0.03, 0)));
    }

    /**
     * Gets the 3D relative pose of the inner component of the lead screw based on
     * its position, for use in AdvantageScope.
     * 
     * @return the 3D pose of the inner component of the lead screw
     */
    public Pose3d getInnerLeadScrewComponentPose() {
        return getOuterLeadScrewComponentPose().plus(OUTER_LEAD_SCREW_TO_INNER_LEAD_SCREW)
                .plus(new Transform3d(getPosition(), 0, 0, new Rotation3d()));
    }

    @Override
    @Log
    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    @Log
    public double getAngle() {
        return getShooterAngle(getPosition());
    }

    @Log
    public double getVelocity() {
        if (RobotBase.isReal())
            return motor.getEncoder().getVelocity();
        else
            return simVelocity;
    }

    @Override
    public void resetPosition() {
        motor.getEncoder().setPosition(0);
        initialized = true;
    }

    /**
     * Sets the voltage of the motors after applying limits.
     * 
     * <p>
     * Limits include: (1) soft stops between MIN_HEIGHT_METERS and
     * MAX_HEIGHT_METERS, (2) upwards movement lock-out if the shooter is in the
     * path of the climber, and (3) total lock-out if the robot is not initialized.
     * 
     * @param voltage the voltage to set the motors to
     */
    @Override
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        if (!GlobalStates.MECH_LIMITS_DISABLED.enabled())
            voltage = Utils.applySoftStops(voltage, getPosition(), MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);

        if (!GlobalStates.INTER_SUBSYSTEM_SAFETIES_DISABLED.enabled()) {
            if (positionTracker.getPosition("climber") < 1.03 && voltage > 0) {
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
        }).withName("shooterTilt.moveToCurrentGoal");
    }

    @Override
    public Command moveToPositionCommand(Supplier<ShooterTiltPosition> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(getLinearActuatorLength(goalPositionSupplier.get().value))),
                moveToCurrentGoalCommand()
                        .until(() -> pidController.atGoal() || GlobalStates.AT_GOAL_OVERRIDE.enabled()))
                .withTimeout(2)
                .withName("shooterTilt.moveToPosition");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                Commands.parallel(
                        Commands.run(() -> pidController
                                .setGoal(getLinearActuatorLength(goalPositionSupplier.get()))),
                        moveToCurrentGoalCommand()))
                .withName("shooterTilt.moveToArbitraryPosition");
    }

    /**
     * Creates a command that angles toe shooter to match the distance
     * from the robot to the target within the speaker.
     * 
     * @param robotPoseSupplier a supplier for the robot's pose
     * @return the command
     */
    public Command targetSpeakerCommand(Supplier<Pose2d> robotPoseSupplier) {
        return targetSpeakerCommand(robotPoseSupplier,
                () -> DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red
                                ? Reflector.reflectPose3d(FieldConstants.SPEAKER_TARGET,
                                        FieldConstants.FIELD_LENGTH)
                                : FieldConstants.SPEAKER_TARGET);
    }

    /**
     * Creates a command that angles the shooter to match the distance
     * from the robot to a specified target.
     * 
     * @param robotPoseSupplier a supplier for the robot's pose
     * @param targetSupplier    a supplier for the target's pose
     * @return the command
     */
    public Command targetSpeakerCommand(Supplier<Pose2d> robotPoseSupplier, Supplier<Pose3d> targetSupplier) {
        return moveToArbitraryPositionCommand(() -> {
            Pose3d target = targetSupplier.get();
            Transform3d diff = new Pose3d(robotPoseSupplier.get()).minus(target);
            this.distance = new Translation2d(diff.getX(), diff.getY()).getNorm();
            this.beyondMaxDistance = distance > MAX_SHOOTING_DISTANCE;
            return getShooterAngle(LEAD_SCREW_HEIGHT_INTERPOLATOR.get(distance));
        }).withName("shooterTilt.targetSpeaker");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(getLinearActuatorLength(getAngle() +
                        delta.get()))),
                moveToCurrentGoalCommand()
                        .until(() -> pidController.atGoal() || GlobalStates.AT_GOAL_OVERRIDE.enabled()))
                .withName("shooterTilt.movePositionDelta");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> pidController.setGoal(getPosition()))
                .andThen(moveToCurrentGoalCommand())
                .withName("shooterTilt.holdCurrentPosition");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition)
                .ignoringDisable(true)
                .withName("shooterTilt.resetPosition");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
                .withName("shooterTilt.setOverriddenSpeed");
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
                })
                .ignoringDisable(true)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("shooterTilt.coastMotors");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction).withName("shooterTilt.sysIdQuasistatic");
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction).withName("shooterTilt.sysIdDynamic");
    }

    @Log
    public boolean atGoal() {
        return (pidController.atSetpoint()
                && Math.abs(pidController.getGoal().position - pidController.getSetpoint().position) <= TOLERANCE
                && !beyondMaxDistance)

                || GlobalStates.AT_GOAL_OVERRIDE.enabled();
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
        }).withName("shooterTilt.setInitialized");
    }

    /**
     * Creates a command that slowly moves the shooter down to the hard stop, and
     * initializes when a current spike is detected.
     * 
     * @apiNote currently unused
     * @return the command
     */
    public Command zeroMechanismCommand() {
        return run(() -> {
            motor.setVoltage(-1);
        }).until(new Trigger(() -> motor.getOutputCurrent() > 20).debounce(0.5))
                .andThen(resetPositionCommand());
    }
}
