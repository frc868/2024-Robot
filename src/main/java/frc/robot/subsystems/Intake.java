package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.techhounds.houndutil.houndlib.SparkConfigurator;
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Intake.IntakePosition;
import static frc.robot.Constants.Intake.*;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

/*
 * The Intake subsystem, which pulls in a note and can be sent to a shooter or can be used to score the amp. 
 * @author rb, jq, ak
 */
@LoggedObject
public class Intake extends SubsystemBase implements BaseSingleJointedArm<IntakePosition> {
    @Log
    private final CANSparkFlex primaryArmMotor;
    @Log
    private final CANSparkFlex secondaryArmMotor;

    @Log
    private final CANSparkFlex rollerMotor;

    @Log
    private final DigitalInput intakeBeam = new DigitalInput(INTAKE_BEAM_ID);
    @Log
    private final DigitalInput shooterPrimaryBeam = new DigitalInput(PRIMARY_SHOOTER_BEAM_ID);
    @Log
    private final DigitalInput shooterSecondaryBeam = new DigitalInput(SECONDARY_SHOOTER_BEAM_ID);

    @Log(groups = "control")
    private final ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, MOVEMENT_CONSTRAINTS);

    @Log(groups = "control")
    private final ArmFeedforward feedforwardController = new ArmFeedforward(kS, kG, kV, kA);

    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            MOTOR_GEARBOX_REPR,
            GEARING,
            MOMENT_OF_INERTIA_KG_METERS_SQUARED,
            LENGTH_METERS,
            MIN_ANGLE_RADIANS,
            MAX_ANGLE_RADIANS,
            true,
            0);

    private final DIOSim intakeBeamSim = new DIOSim(intakeBeam);
    private final DIOSim shooterPrimaryBeamSim = new DIOSim(shooterPrimaryBeam);
    private final DIOSim shooterSecondaryBeamSim = new DIOSim(shooterSecondaryBeam);

    @Log(groups = "control")
    private double feedbackVoltage = 0;
    @Log(groups = "control")
    private double feedforwardVoltage = 0;

    private double simVelocity = 0.0;

    private final MutableMeasure<Voltage> sysidAppliedVoltageMeasure = MutableMeasure.mutable(Volts.of(0));
    private final MutableMeasure<Angle> sysidPositionMeasure = MutableMeasure.mutable(Radians.of(0));
    private final MutableMeasure<Velocity<Angle>> sysidVelocityMeasure = MutableMeasure.mutable(RadiansPerSecond.of(0));

    private final SysIdRoutine sysIdRoutine;

    public Intake() {
        primaryArmMotor = SparkConfigurator.createSparkFlex(PRIMARY_ARM_MOTOR_ID, MotorType.kBrushless, false,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(ARM_CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_RADIANS),
                (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_RADIANS / 60.0));

        secondaryArmMotor = SparkConfigurator.createSparkFlex(SECONDARY_ARM_MOTOR_ID, MotorType.kBrushless, false,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(ARM_CURRENT_LIMIT),
                (s) -> s.follow(primaryArmMotor));

        rollerMotor = SparkConfigurator.createSparkFlex(ROLLER_MOTOR_ID, MotorType.kBrushless, false,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(ROLLER_CURRENT_LIMIT));

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.of(0.5).per(Seconds.of(1)), Volts.of(2), null, null),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> setVoltage(volts.magnitude()),
                        log -> {
                            log.motor("primary")
                                    .voltage(sysidAppliedVoltageMeasure.mut_replace(primaryArmMotor.getAppliedOutput(),
                                            Volts))
                                    .angularPosition(sysidPositionMeasure.mut_replace(getPosition(), Radians))
                                    .angularVelocity(sysidVelocityMeasure.mut_replace(getVelocity(), RadiansPerSecond));
                        },
                        this));

        pidController.setGoal(IntakePosition.STOW.value);
        intakeBeamSim.setValue(false);
        shooterPrimaryBeamSim.setValue(false);
        shooterSecondaryBeamSim.setValue(false);
        setDefaultCommand(moveToCurrentGoalCommand());
    }

    @Override
    public void simulationPeriodic() {
        armSim.setInput(primaryArmMotor.getAppliedOutput());
        armSim.update(0.020);
        primaryArmMotor.getEncoder().setPosition(armSim.getAngleRads());
        simVelocity = armSim.getVelocityRadPerSec();
    }

    @Log
    public Pose3d getComponentPose() {
        return BASE_COMPONENT_POSE.plus(new Transform3d(0, 0, 0, new Rotation3d(0, -getPosition(), 0)));
    }

    @Override
    @Log
    public double getPosition() {
        return primaryArmMotor.getEncoder().getPosition();
    }

    @Log
    public double getVelocity() {
        if (RobotBase.isReal())
            return primaryArmMotor.getEncoder().getVelocity();
        else
            return simVelocity;
    }

    @Override
    public void resetPosition() {
        primaryArmMotor.getEncoder().setPosition(0);
    }

    @Override
    public void setVoltage(double voltage) {
        primaryArmMotor.setVoltage(voltage > 12 ? 12 : voltage);
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
                moveToCurrentGoalCommand().until(pidController::atGoal))
                .withName("intake.moveToPosition");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get())),
                moveToCurrentGoalCommand().until(pidController::atGoal))
                .withName("intake.moveToArbitraryPosition");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(getPosition() + delta.get())),
                moveToCurrentGoalCommand().until(pidController::atGoal)).withTimeout(2)
                .withName("intake.movePositionDelta");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> pidController.setGoal(getPosition()))
                .andThen(moveToCurrentGoalCommand()).withName("intake.holdCurrentPosition");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("intake.resetPosition");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return run(() -> setVoltage(12.0 * speed.get()))
                .withName("intake.setOverriddenSpeed");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> primaryArmMotor.stopMotor())
                .andThen(() -> {
                    primaryArmMotor.setIdleMode(IdleMode.kCoast);
                    secondaryArmMotor.setIdleMode(IdleMode.kCoast);
                })
                .finallyDo((d) -> {
                    primaryArmMotor.setIdleMode(IdleMode.kBrake);
                    secondaryArmMotor.setIdleMode(IdleMode.kBrake);
                    pidController.reset(getPosition());
                })
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("intake.coastMotors");
    }

    public Command runRollersCommand() {
        return Commands.startEnd(
                () -> rollerMotor.setVoltage(3),
                () -> rollerMotor.setVoltage(0))
                .withName("intake.runRollers");
    }

    public Command reverseRollersCommand() {
        return Commands.startEnd(
                () -> rollerMotor.setVoltage(-3),
                () -> rollerMotor.setVoltage(0))
                .withName("intake.reverseRollers");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction).withName("intake.sysIdQuasistatic");
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction).withName("intake.sysIdQuasistatic");
    }

    public Command intakeNoteCommand() {
        return Commands.sequence(
                moveToPositionCommand(() -> IntakePosition.GROUND),
                moveToCurrentGoalCommand().alongWith(runRollersCommand()).until(shooterPrimaryBeam::get),
                moveToPositionCommand(() -> IntakePosition.STOW)).withName("intake.intakeNote");
    }

    public Command intakeNoteAutoCommand() {
        return Commands.sequence(
                moveToPositionCommand(() -> IntakePosition.GROUND),
                moveToCurrentGoalCommand().alongWith(runRollersCommand()).until(shooterPrimaryBeam::get))
                .withName("intake.intakeNoteAuto");
    }

    public Command ampScoreCommand(BooleanSupplier runRollers) {
        return Commands.sequence(
                moveToPositionCommand(() -> IntakePosition.GROUND),
                moveToCurrentGoalCommand().alongWith(reverseRollersCommand()).until(intakeBeam::get),
                moveToPositionCommand(() -> IntakePosition.AMP),
                moveToCurrentGoalCommand().until(runRollers),
                moveToCurrentGoalCommand().alongWith(reverseRollersCommand()).withTimeout(1),
                moveToPositionCommand(() -> IntakePosition.STOW)).withName("intake.ampScore");
    }

    public Command simTriggerIntakeBeamCommand() {
        return Commands.runOnce(() -> intakeBeamSim.setValue(true))
                .andThen(Commands.waitSeconds(1))
                .andThen(Commands.runOnce(() -> intakeBeamSim.setValue(false)))
                .withName("intake.simTriggerIntakeBeam");
    }

    public Command simTriggerShooterBeamCommand() {
        return Commands.runOnce(() -> shooterPrimaryBeamSim.setValue(true))
                .andThen(Commands.waitSeconds(1))
                .andThen(Commands.runOnce(() -> shooterPrimaryBeamSim.setValue(false)))
                .withName("intake.simTriggerShooterBeam");
    }
}
