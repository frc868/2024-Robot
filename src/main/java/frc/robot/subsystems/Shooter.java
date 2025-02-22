package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.techhounds.houndutil.houndauto.Reflector;
import com.techhounds.houndutil.houndlib.SparkConfigurator;
import com.techhounds.houndutil.houndlib.subsystems.BaseShooter;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Shooter.*;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants;
import frc.robot.GlobalStates;

/**
 * The shooter subsystem, used to shoot notes into the speaker. Controls the two
 * motors powering the flywheel.
 */
@LoggedObject
public class Shooter extends SubsystemBase implements BaseShooter {
    @Log
    private final CANSparkFlex leftMotor;

    @Log
    private final CANSparkFlex rightMotor;

    @Log(groups = "control")
    private final PIDController leftPidController = new PIDController(left_kP, left_kI, left_kD);
    @Log(groups = "control")
    private final PIDController rightPidController = new PIDController(right_kP, right_kI, right_kD);

    @Log(groups = "control")
    private final SimpleMotorFeedforward leftFeedforwardController = new SimpleMotorFeedforward(left_kS, left_kV,
            left_kA);
    @Log(groups = "control")
    private final SimpleMotorFeedforward rightFeedforwardController = new SimpleMotorFeedforward(right_kS, right_kV,
            right_kA);

    private final FlywheelSim flywheelSim = new FlywheelSim(MOTOR_GEARBOX_REPR, GEARING,
            MOMENT_OF_INERTIA_KG_METERS_SQUARED);

    @Log(groups = "control")
    private double leftFeedforwardVoltage = 0.0;
    @Log(groups = "control")
    private double rightFeedforwardVoltage = 0.0;
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
                (s) -> s.getEncoder().setVelocityConversionFactor(1.0 / 60.0),
                (s) -> s.getEncoder().setAverageDepth(2),
                (s) -> s.getEncoder().setMeasurementPeriod(16));

        rightMotor = SparkConfigurator.createSparkFlex(RIGHT_MOTOR_ID, MotorType.kBrushless, true,
                (s) -> s.setIdleMode(IdleMode.kCoast),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(1.0),
                (s) -> s.getEncoder().setVelocityConversionFactor(1.0 / 60.0),
                (s) -> s.getEncoder().setAverageDepth(2),
                (s) -> s.getEncoder().setMeasurementPeriod(16));

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> setVoltage(volts.magnitude()),
                        log -> {
                            log.motor("left")
                                    .voltage(sysidAppliedVoltageMeasure.mut_replace(leftMotor.getAppliedOutput(),
                                            Volts))
                                    .angularPosition(sysidPositionMeasure
                                            .mut_replace(leftMotor.getEncoder().getPosition(), Rotations))
                                    .angularVelocity(
                                            sysidVelocityMeasure.mut_replace(leftMotor.getEncoder().getVelocity(),
                                                    RotationsPerSecond));
                            log.motor("right")
                                    .voltage(sysidAppliedVoltageMeasure.mut_replace(rightMotor.getAppliedOutput(),
                                            Volts))
                                    .angularPosition(sysidPositionMeasure
                                            .mut_replace(rightMotor.getEncoder().getPosition(), Rotations))
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
            leftFeedforwardVoltage = leftFeedforwardController.calculate(goalVelocitySupplier.get());
            rightFeedforwardVoltage = rightFeedforwardController.calculate(goalVelocitySupplier.get());
            setLeftVoltage(leftFeedbackVoltage + leftFeedforwardVoltage);
            setRightVoltage(rightFeedbackVoltage + rightFeedforwardVoltage);
        }).withName("shooter.spinAtVelocity");
    }

    /**
     * Creates a command that spins the shooter at a velocity to match the distance
     * from the robot to the target within the speaker.
     * 
     * @param robotPoseSupplier a supplier for the robot's pose
     * @return the command
     */
    public Command targetSpeakerCommand(Supplier<Pose2d> robotPoseSupplier) {
        return targetPoseCommand(robotPoseSupplier,
                () -> DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red
                                ? Reflector.reflectPose3d(FieldConstants.SPEAKER_TARGET,
                                        FieldConstants.FIELD_LENGTH)
                                : FieldConstants.SPEAKER_TARGET);
    }

    /**
     * Creates a command that spins the shooter at a velocity to match the distance
     * from the robot to a specified target.
     * 
     * @param robotPoseSupplier a supplier for the robot's pose
     * @param targetSupplier    a supplier for the target's pose
     * @return the command
     */
    public Command targetPoseCommand(Supplier<Pose2d> robotPoseSupplier, Supplier<Pose3d> targetSupplier) {
        return spinAtVelocityCommand(() -> {
            Pose3d target = targetSupplier.get();
            Transform3d diff = new Pose3d(robotPoseSupplier.get()).minus(target);
            return SPEED_INTERPOLATOR.get(new Translation2d(diff.getX(), diff.getY()).getNorm());
        }).withName("shooter.targetPose");
    }

    /**
     * Creates a command that holds the shooter at a specified velocity (for
     * idling). Will reach the target velocity using PID control, but if the current
     * velocity is significantly higher than the target velocity, the shooter is
     * allowed to ramp down on its own without braking, to conserve power.
     * 
     * @param goalVelocitySupplier a supplier for the target velocity
     * @return the command
     */
    public Command holdVelocityCommand(Supplier<Double> goalVelocitySupplier) {
        return run(() -> {
            if (getVelocity() > goalVelocitySupplier.get() * 1.2) {
                setVoltage(0);
            } else {
                leftFeedforwardVoltage = leftFeedforwardController.calculate(goalVelocitySupplier.get());
                rightFeedforwardVoltage = rightFeedforwardController.calculate(goalVelocitySupplier.get());
                leftFeedbackVoltage = leftPidController.calculate(getLeftVelocity(),
                        goalVelocitySupplier.get());
                rightFeedbackVoltage = rightPidController.calculate(getRightVelocity(),
                        goalVelocitySupplier.get());
                setLeftVoltage(leftFeedforwardVoltage + leftFeedbackVoltage);
                setRightVoltage(rightFeedforwardVoltage + rightFeedbackVoltage);
            }

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

    public Command stopCommand() {
        return run(this::stop);
    }

    @Log
    public boolean atGoal() {
        return (leftPidController.atSetpoint() && rightPidController.atSetpoint())
                || GlobalStates.AT_GOAL_OVERRIDE.enabled();
    }
}
