package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.techhounds.houndutil.houndlib.SparkConfigurator;
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterTilt.ShooterTiltPosition;
import frc.robot.FieldConstants;

import static frc.robot.Constants.ShooterTilt.*;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

@LoggedObject
public class ShooterTilt extends SubsystemBase implements BaseSingleJointedArm<ShooterTiltPosition> {
    @Log
    private final CANSparkFlex motor;

    @Log
    private final ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, MOVEMENT_CONSTRAINTS);

    @Log
    private final ElevatorFeedforward feedforwardController = new ElevatorFeedforward(kS,
            kG, kV, kA);

    @Log
    private final ElevatorSim elevatorSim = new ElevatorSim(
            MOTOR_GEARBOX_REPR,
            GEARING / ENCODER_ROTATIONS_TO_METERS,
            MASS_KG,
            1 / (2.0 * Math.PI),
            MIN_HEIGHT_METERS,
            MAX_HEIGHT_METERS,
            true,
            0);

    @Log(groups = "control")
    private double feedbackVoltage = 0;
    @Log(groups = "control")
    private double feedforwardVoltage = 0;

    @Log
    private double simVelocity = 0.0;

    private final MutableMeasure<Voltage> sysidAppliedVoltageMeasure = MutableMeasure.mutable(Volts.of(0));
    private final MutableMeasure<Distance> sysidPositionMeasure = MutableMeasure.mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> sysidVelocityMeasure = MutableMeasure
            .mutable(MetersPerSecond.of(0));

    private final SysIdRoutine sysIdRoutine;

    public ShooterTilt() {
        motor = SparkConfigurator.createSparkFlex(MOTOR_ID,
                MotorType.kBrushless, false,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS),
                (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS /
                        60.0));

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

        pidController.setTolerance(0.01);

        setDefaultCommand(moveToCurrentGoalCommand());
    }

    @Override
    public void simulationPeriodic() {
        elevatorSim.setInput(motor.getAppliedOutput());
        elevatorSim.update(0.020);
        motor.getEncoder().setPosition(elevatorSim.getPositionMeters());
        simVelocity = elevatorSim.getVelocityMetersPerSecond();
    }

    @Log
    public Pose3d getShooterComponentPose() {
        return BASE_SHOOTER_POSE.plus(new Transform3d(0, 0, 0, new Rotation3d(0, -getAngle(), 0)));
    }

    @Log
    public Pose3d getOuterLeadScrewComponentPose() {
        // small offset of 0.03rad added to fix an alignment issue between the two;
        // since this is constant, it's likely a CAD zeroing problem
        return BASE_OUTER_LEAD_SCREW_POSE
                .plus(new Transform3d(0, 0, 0, new Rotation3d(0, -getLeadScrewAngle(getPosition()) + 0.03, 0)));
    }

    @Log
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
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage > 12 ? 12 : voltage);
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
                moveToCurrentGoalCommand().until(pidController::atGoal))
                .withName("shooterTilt.moveToPosition");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                Commands.parallel(
                        Commands.run(() -> pidController.setGoal(getLinearActuatorLength(goalPositionSupplier.get()))),
                        moveToCurrentGoalCommand()))
                .withName("shooterTilt.moveToArbitraryPosition");
    }

    public Command targetSpeakerCommand(Supplier<Pose2d> poseSupplier) {
        return moveToArbitraryPositionCommand(() -> {
            Transform3d diff = new Pose3d(poseSupplier.get()).minus(FieldConstants.TARGET);
            return new Rotation2d(Math.abs(diff.getX()), Math.abs(diff.getZ())).getRadians();
        }).withName("shooterTilt.targetSpeaker");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(getLinearActuatorLength(getAngle() +
                        delta.get()))),
                moveToCurrentGoalCommand().until(pidController::atGoal))
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
        return runOnce(this::resetPosition).withName("shooterTilt.resetPosition");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return run(() -> setVoltage(12.0 * speed.get()))
                .withName("shooterTilt.setOverriddenSpeed");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(motor::stopMotor)
                .andThen(() -> motor.setIdleMode(IdleMode.kCoast))
                .finallyDo((d) -> {
                    motor.setIdleMode(IdleMode.kBrake);
                    pidController.reset(getPosition());
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("shooterTilt.coastMotors");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction).withName("shooterTilt.sysIdQuasistatic");
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction).withName("shooterTilt.sysIdDynamic");
    }

    public boolean atGoal() {
        return pidController.atGoal();
    }
}
