package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.techhounds.houndutil.houndlib.PositionTracker;
import com.techhounds.houndutil.houndlib.SparkConfigurator;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.GlobalStates;
import frc.robot.Constants.Intake.IntakePosition;
import static frc.robot.Constants.Intake.*;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

/**
 * The Intake subsystem, which pulls in a note and can be sent to a shooter or
 * can be used to score the amp. Handles sensing the position of the note in the
 * intake and moving it to requested positions.
 * 
 * @author rb, jq, ak
 */
@LoggedObject
public class Intake extends SubsystemBase implements BaseSingleJointedArm<IntakePosition>, BaseIntake {
    @Log
    private final CANSparkFlex leftArmMotor;
    @Log
    private final CANSparkFlex rightArmMotor;

    @Log
    private final CANSparkFlex rollerMotor;

    /**
     * The beam sensor within the entrance to the intake (1st beam passed by a
     * note).
     */
    @Log
    private final DigitalInput intakeBeam = new DigitalInput(INTAKE_BEAM_ID);
    /**
     * The beam sensor closer to the intake, within the shooter's storage (2nd beam
     * passed by a note).
     */
    @Log
    private final DigitalInput shooterCloseBeam = new DigitalInput(SHOOTER_CLOSE_BEAM_ID);
    /**
     * The beam sensor closer to the shooter wheels, within the shooter's storage
     * (3rd beam passed by a note).
     */
    @Log
    private final DigitalInput shooterFarBeam = new DigitalInput(SHOOTER_FAR_BEAM_ID);

    @Log(groups = "control")
    private final ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, MOVEMENT_CONSTRAINTS);

    private final ArmFeedforward feedforwardController = new ArmFeedforward(kS, kG, kV, kA);

    /** The representation of the "arm" for simulation. */
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            MOTOR_GEARBOX_REPR,
            GEARING,
            MOMENT_OF_INERTIA_KG_METERS_SQUARED,
            LENGTH_METERS,
            MIN_ANGLE_RADIANS,
            MAX_ANGLE_RADIANS,
            true,
            MAX_ANGLE_RADIANS);

    private final DIOSim intakeBeamSim = new DIOSim(intakeBeam);
    private final DIOSim shooterFarBeamSim = new DIOSim(shooterFarBeam);
    private final DIOSim shooterCloseBeamSim = new DIOSim(shooterCloseBeam);

    @Log(groups = "control")
    private double feedbackVoltage = 0;
    @Log(groups = "control")
    private double feedforwardVoltage = 0;

    private double simVelocity = 0.0;

    private final MutableMeasure<Voltage> sysidAppliedVoltageMeasure = MutableMeasure.mutable(Volts.of(0));
    private final MutableMeasure<Angle> sysidPositionMeasure = MutableMeasure.mutable(Radians.of(0));
    private final MutableMeasure<Velocity<Angle>> sysidVelocityMeasure = MutableMeasure.mutable(RadiansPerSecond.of(0));

    private final SysIdRoutine sysIdRoutine;

    /**
     * Trigger that becomes active when a note is is in the shooter.
     */
    public final Trigger noteInShooterTrigger = new Trigger(shooterCloseBeam::get).negate();
    /**
     * Trigger that becomes active when a note is fully in the shooter.
     */
    public final Trigger noteFullyInShooterTrigger = new Trigger(shooterFarBeam::get).negate();

    private boolean prevIntakeBeamState = true;

    /**
     * Trigger that becomes active when a note successfully moves from the shooter
     * to the intake, identified by the intake beam moving from off -> on.
     */
    public final Trigger noteInIntakeFromShooterTrigger = new Trigger(() -> {
        boolean triggered = !prevIntakeBeamState && intakeBeam.get();
        prevIntakeBeamState = intakeBeam.get();
        return triggered;
    });
    /**
     * Trigger that becomes active when a note enters the intake from the outside.
     */
    public final Trigger noteInIntakeFromOutsideTrigger = new Trigger(intakeBeam::get).negate();

    @Log
    private boolean initialized = false;

    @SuppressWarnings("unused")
    private PositionTracker positionTracker;

    public Intake(PositionTracker positionTracker) {
        leftArmMotor = SparkConfigurator.createSparkFlex(PRIMARY_ARM_MOTOR_ID, MotorType.kBrushless, true,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(ARM_CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_RADIANS),
                (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_RADIANS / 60.0));

        rightArmMotor = SparkConfigurator.createSparkFlex(SECONDARY_ARM_MOTOR_ID, MotorType.kBrushless, true,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(ARM_CURRENT_LIMIT),
                (s) -> s.follow(leftArmMotor, true));

        rollerMotor = SparkConfigurator.createSparkFlex(ROLLER_MOTOR_ID, MotorType.kBrushless, true,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(ROLLER_CURRENT_LIMIT));

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.of(1).per(Seconds.of(1)), Volts.of(3), null, null),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> setVoltage(volts.magnitude()),
                        log -> {
                            log.motor("primary")
                                    .voltage(sysidAppliedVoltageMeasure.mut_replace(leftArmMotor.getAppliedOutput(),
                                            Volts))
                                    .angularPosition(sysidPositionMeasure.mut_replace(getPosition(), Radians))
                                    .angularVelocity(sysidVelocityMeasure.mut_replace(getVelocity(), RadiansPerSecond));
                        },
                        this));

        this.positionTracker = positionTracker;

        pidController.setTolerance(TOLERANCE);
        pidController.setGoal(IntakePosition.GROUND.value);

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                pidController.reset(getPosition());
            } catch (Exception e) {
                e.printStackTrace(System.err);
            }
        }).start();

        intakeBeamSim.setValue(true);
        shooterFarBeamSim.setValue(true);
        shooterCloseBeamSim.setValue(true);
        setDefaultCommand(moveToCurrentGoalCommand());
    }

    /**
     * Updated the physics simulation, and sets data on motor controllers based on
     * its results.
     */
    @Override
    public void simulationPeriodic() {
        armSim.setInput(leftArmMotor.getAppliedOutput());
        armSim.update(0.020);
        leftArmMotor.getEncoder().setPosition(armSim.getAngleRads());
        simVelocity = armSim.getVelocityRadPerSec();
    }

    /**
     * Gets the 3D relative pose of the intake based on its position, for use in
     * AdvantageScope.
     * 
     * @return the 3D pose of the intake
     */
    public Pose3d getComponentPose() {
        return BASE_COMPONENT_POSE.plus(new Transform3d(0, 0, 0, new Rotation3d(0, -getPosition(), 0)));
    }

    @Override
    @Log
    public double getPosition() {
        return leftArmMotor.getEncoder().getPosition();
    }

    @Log
    public double getVelocity() {
        if (RobotBase.isReal())
            return leftArmMotor.getEncoder().getVelocity();
        else
            return simVelocity;
    }

    @Override
    public void resetPosition() {
        leftArmMotor.getEncoder().setPosition(IntakePosition.TOP.value);
        initialized = true;
    }

    /**
     * Sets the voltage of the motors after applying limits.
     * 
     * <p>
     * Limits include: (1) soft stops between MIN_ANGLE_RADIANS and
     * MIN_ANGLE_RADIANS and (2) total lock-out if the robot is not initialized.
     * 
     * @param voltage the voltage to set the motors to
     */
    @Override
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        if (!GlobalStates.MECH_LIMITS_DISABLED.enabled())
            voltage = Utils.applySoftStops(voltage, getPosition(), MIN_ANGLE_RADIANS, MAX_ANGLE_RADIANS - 0.03);

        if (!GlobalStates.INITIALIZED.enabled() && !GlobalStates.INTER_SUBSYSTEM_SAFETIES_DISABLED.enabled()) {
            voltage = 0.0;
        }
        leftArmMotor.setVoltage(voltage);
    }

    public void setRollerVoltage(double voltage) {
        rollerMotor.setVoltage(voltage);
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            feedbackVoltage = pidController.calculate(getPosition());
            feedforwardVoltage = feedforwardController.calculate(getPosition(),
                    pidController.getSetpoint().velocity);
            setVoltage(feedbackVoltage + feedforwardVoltage);
        }).withName("intake.moveToCurrentGoal");
    }

    @Override
    public Command moveToPositionCommand(Supplier<IntakePosition> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get().value)),
                moveToCurrentGoalCommand().until(this::atGoal)).withTimeout(2)
                .withName("intake.moveToPosition");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get())),
                moveToCurrentGoalCommand().until(this::atGoal))
                .withName("intake.moveToArbitraryPosition");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(getPosition() + delta.get())),
                moveToCurrentGoalCommand().until(this::atGoal)).withTimeout(2)
                .withName("intake.movePositionDelta");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> pidController.setGoal(getPosition()))
                .andThen(moveToCurrentGoalCommand()).withName("intake.holdCurrentPosition");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition)
                .ignoringDisable(true)
                .withName("intake.resetPosition");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
                .withName("intake.setOverriddenSpeed");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> leftArmMotor.stopMotor())
                .andThen(() -> {
                    leftArmMotor.setIdleMode(IdleMode.kCoast);
                    rightArmMotor.setIdleMode(IdleMode.kCoast);
                })
                .finallyDo((d) -> {
                    leftArmMotor.setIdleMode(IdleMode.kBrake);
                    rightArmMotor.setIdleMode(IdleMode.kBrake);
                    pidController.reset(getPosition());
                })
                .ignoringDisable(true)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("intake.coastMotors");
    }

    @Override
    public Command runRollersCommand() {
        return Commands.startEnd(
                () -> setRollerVoltage(12),
                () -> setRollerVoltage(0))
                .withName("intake.runRollers");
    }

    @Override
    public Command reverseRollersCommand() {
        return Commands.startEnd(
                () -> setRollerVoltage(-12),
                () -> setRollerVoltage(0))
                .withName("intake.reverseRollers");
    }

    /**
     * Command to run the intake rollers during autonomous (with a safer voltage
     * than teleop).
     * 
     * @apiNote this function is technically no longer necessary, as the final
     *          iteration at Worlds had used the same voltage for both teleop and
     *          auto.
     * @return the command
     */
    public Command runRollersAutoCommand() {
        return Commands.startEnd(
                () -> setRollerVoltage(12),
                () -> setRollerVoltage(0))
                .withName("intake.runRollers");
    }

    /**
     * Command to run the intake rollers at half speed, used when moving a note from
     * the 1st to the 2nd beam.
     * 
     * @return the command
     */
    public Command runRollersHalfCommand() {
        return Commands.startEnd(
                () -> setRollerVoltage(6),
                () -> setRollerVoltage(0))
                .withName("intake.runRollersHalf");
    }

    /**
     * Command to run the intake rollers at a slow speed, used when moving a note
     * from the 2nd to the 3rd beam.
     * 
     * @return the command
     */
    public Command runRollersSlowCommand() {
        return Commands.startEnd(
                () -> setRollerVoltage(2),
                () -> setRollerVoltage(0))
                .withName("intake.runRollersSlow");
    }

    /**
     * Command to run the rollers at an appropriate speed for intaking a note from
     * the source.
     * 
     * @return the command
     */
    public Command sourceIntakeRollersCommand() {
        return Commands.startEnd(
                () -> setRollerVoltage(5),
                () -> setRollerVoltage(0))
                .withName("intake.sourceIntakeRollers");
    }

    /**
     * Command to run the rollers at an appropriate speed for scoring in the amp.
     * 
     * @return the command
     */
    public Command ampScoreRollersCommand() {
        return Commands.startEnd(
                () -> setRollerVoltage(-3.75),
                () -> setRollerVoltage(0))
                .withName("intake.ampScoreRollers");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction).withName("intake.sysIdQuasistatic");
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction).withName("intake.sysIdQuasistatic");
    }

    /**
     * Compound command that moves the intake down, intakes a note from the ground,
     * moves it to the stowed location in the shooter, and stows the intake when
     * appropriate.
     * 
     * @return the command
     */
    public Command intakeNoteCommand() {
        return Commands.sequence(
                moveToPositionCommand(() -> IntakePosition.GROUND),
                moveToCurrentGoalCommand().alongWith(runRollersCommand())
                        .until(noteInIntakeFromOutsideTrigger.or(noteInShooterTrigger)),
                moveToCurrentGoalCommand().alongWith(runRollersHalfCommand())
                        .until(noteInShooterTrigger),
                moveToPositionCommand(() -> IntakePosition.STOW).andThen(moveToCurrentGoalCommand())
                        .alongWith(runRollersSlowCommand())
                        .until(noteFullyInShooterTrigger))
                .withName("intake.intakeNote");
    }

    /**
     * Compound command that moves the intake down (and keeps it there), intakes a
     * note, and selectively triggers portions of the intaking sequence depending on
     * where the note is detected.
     * 
     * @return the command
     */
    public Command intakeNoteAutoCommand() {
        return Commands.parallel(
                moveToPositionCommand(() -> IntakePosition.GROUND),
                Commands.sequence(
                        runRollersAutoCommand()
                                .unless(noteInIntakeFromOutsideTrigger.or(noteInShooterTrigger)
                                        .or(noteFullyInShooterTrigger))
                                .until(noteInIntakeFromOutsideTrigger.or(noteInShooterTrigger)
                                        .or(noteFullyInShooterTrigger)),
                        runRollersHalfCommand()
                                .unless(noteInShooterTrigger.or(noteFullyInShooterTrigger))
                                .until(noteInShooterTrigger.or(noteFullyInShooterTrigger)),
                        runRollersSlowCommand()
                                .unless(noteFullyInShooterTrigger)
                                .until(noteFullyInShooterTrigger)))
                .withName("intake.intakeNoteAuto");
    }

    /**
     * Command that intakes a note from the source until it is detected, then stops
     * the rollers and stows the intake.
     * 
     * @return the command
     */
    public Command intakeFromSourceCommand() {
        return Commands.sequence(
                moveToPositionCommand(() -> IntakePosition.SOURCE),
                moveToCurrentGoalCommand().alongWith(sourceIntakeRollersCommand())
                        .until(noteInIntakeFromOutsideTrigger),
                moveToPositionCommand(() -> IntakePosition.STOW)).withName("intake.intakeNote");
    }

    /**
     * Command that simulates the the intake beam being triggered. Used in
     * simulation to validate autonomous commands.
     * 
     * @return the command
     */
    public Command simTriggerIntakeBeamCommand() {
        return Commands.runOnce(() -> intakeBeamSim.setValue(false))
                .andThen(Commands.waitSeconds(1))
                .andThen(Commands.runOnce(() -> intakeBeamSim.setValue(true)))
                .withName("intake.simTriggerIntakeBeam");
    }

    /**
     * Command that simulates the shooter close beam being triggered. Used in
     * simulation to validate autonomous commands.
     * 
     * @return the command
     */
    public Command simTriggerShooterCloseBeamCommand() {
        return Commands.runOnce(() -> shooterCloseBeamSim.setValue(false))
                .andThen(Commands.waitSeconds(1))
                .andThen(Commands.runOnce(() -> shooterCloseBeamSim.setValue(true)))
                .withName("intake.simTriggerShooterCloseBeam");
    }

    /**
     * Command that simulates the shooter far beam being triggered. Used in
     * simulation to validate autonomous commands.
     * 
     * @return the command
     */
    public Command simTriggerShooterFarBeamCommand() {
        return Commands.runOnce(() -> shooterFarBeamSim.setValue(false))
                .andThen(Commands.waitSeconds(1))
                .andThen(Commands.runOnce(() -> shooterFarBeamSim.setValue(true)))
                .withName("intake.simTriggerShooterFarBeam");
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
        }).withName("intake.setInitialized");
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
            leftArmMotor.setVoltage(1);
            rightArmMotor.setVoltage(1);
        })
                .until(() -> (leftArmMotor.getOutputCurrent() > 20) && (rightArmMotor.getOutputCurrent() > 20))
                .andThen(resetPositionCommand());
    }

    public boolean atGoal() {
        return pidController.atGoal() || GlobalStates.AT_GOAL_OVERRIDE.enabled();
    }
}
