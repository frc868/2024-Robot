package frc.robot.subsystems;

import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndauto.Reflector;
import com.techhounds.houndutil.houndlib.ChassisAccelerations;
import com.techhounds.houndutil.houndlib.MotorHoldMode;
import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive;
import com.techhounds.houndutil.houndlib.swerve.KrakenCoaxialSwerveModule;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants;
import frc.robot.utils.TrajectoryCalcs;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Constants.Teleop.*;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

@LoggedObject
public class Drivetrain extends SubsystemBase implements BaseSwerveDrive {
    @Log(groups = "modules")
    private final KrakenCoaxialSwerveModule frontLeft = new KrakenCoaxialSwerveModule(
            FRONT_LEFT_DRIVE_MOTOR_ID,
            FRONT_LEFT_STEER_MOTOR_ID,
            FRONT_LEFT_STEER_ENCODER_ID,
            CAN_BUS_NAME,
            DRIVE_MOTORS_INVERTED,
            STEER_MOTORS_INVERTED,
            STEER_CANCODERS_INVERTED,
            FRONT_LEFT_OFFSET,
            SWERVE_CONSTANTS);

    @Log(groups = "modules")
    private final KrakenCoaxialSwerveModule frontRight = new KrakenCoaxialSwerveModule(
            FRONT_RIGHT_DRIVE_MOTOR_ID,
            FRONT_RIGHT_STEER_MOTOR_ID,
            FRONT_RIGHT_STEER_ENCODER_ID,
            CAN_BUS_NAME,
            DRIVE_MOTORS_INVERTED,
            STEER_MOTORS_INVERTED,
            STEER_CANCODERS_INVERTED,
            FRONT_RIGHT_OFFSET,
            SWERVE_CONSTANTS);

    @Log(groups = "modules")
    private final KrakenCoaxialSwerveModule backLeft = new KrakenCoaxialSwerveModule(
            BACK_LEFT_DRIVE_MOTOR_ID,
            BACK_LEFT_STEER_MOTOR_ID,
            BACK_LEFT_STEER_ENCODER_ID,
            CAN_BUS_NAME,
            DRIVE_MOTORS_INVERTED,
            STEER_MOTORS_INVERTED,
            STEER_CANCODERS_INVERTED,
            BACK_LEFT_OFFSET,
            SWERVE_CONSTANTS);

    @Log(groups = "modules")
    private final KrakenCoaxialSwerveModule backRight = new KrakenCoaxialSwerveModule(
            BACK_RIGHT_DRIVE_MOTOR_ID,
            BACK_RIGHT_STEER_MOTOR_ID,
            BACK_RIGHT_STEER_ENCODER_ID,
            CAN_BUS_NAME,
            DRIVE_MOTORS_INVERTED,
            STEER_MOTORS_INVERTED,
            STEER_CANCODERS_INVERTED,
            BACK_RIGHT_OFFSET,
            SWERVE_CONSTANTS);

    @Log
    private Pigeon2 pigeon = new Pigeon2(0, CAN_BUS_NAME);

    private SwerveDrivePoseEstimator poseEstimator;

    @Log(groups = "control")
    private ProfiledPIDController driveController = new ProfiledPIDController(
            XY_kP, XY_kI, XY_kD, XY_CONSTRAINTS);

    @Log(groups = "control")
    private double poseDistance = 0.0;

    @Log(groups = "control")
    private ProfiledPIDController rotationController = new ProfiledPIDController(
            THETA_kP, THETA_kI, THETA_kD, THETA_CONSTRAINTS);

    @Log(groups = "control")
    private SwerveModuleState[] commandedModuleStates = new SwerveModuleState[] { new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };

    @Log(groups = "control")
    private ChassisSpeeds commandedChassisSpeeds = new ChassisSpeeds();

    @Log
    private DriveMode driveMode = DriveMode.FIELD_ORIENTED;

    /**
     * Whether to override the inputs of the driver for maintaining or turning to a
     * specific angle.
     */
    @Log
    private boolean isControlledRotationEnabled = false;

    private SwerveDriveOdometry simOdometry;
    private SwerveModulePosition[] lastModulePositions = getModulePositions();

    private final MutableMeasure<Voltage> sysidDriveAppliedVoltageMeasure = mutable(Volts.of(0));
    private final MutableMeasure<Distance> sysidDrivePositionMeasure = mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> sysidDriveVelocityMeasure = mutable(MetersPerSecond.of(0));

    private final SysIdRoutine sysIdDrive;

    private final MutableMeasure<Voltage> sysidSteerAppliedVoltageMeasure = mutable(Volts.of(0));
    private final MutableMeasure<Angle> sysidSteerPositionMeasure = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> sysidSteerVelocityMeasure = mutable(RotationsPerSecond.of(0));

    private final SysIdRoutine sysIdSteer;

    private final Orchestra orchestra = new Orchestra();

    @Log
    private Pose2d targettedStagePose = new Pose2d();
    @Log
    private Pose2d targettedPose = new Pose2d();

    @Log
    private double distance = 0.0;
    @Log
    private ProfiledPIDController yPidController = new ProfiledPIDController(XY_kP, XY_kI, XY_kD, XY_CONSTRAINTS);

    private ChassisSpeeds prevFieldRelVelocities = new ChassisSpeeds();

    @Log
    private double effectiveWheelRadius = 0.0;

    /** Initializes the drivetrain. */
    public Drivetrain() {
        poseEstimator = new SwerveDrivePoseEstimator(
                KINEMATICS,
                getRotation(),
                getModulePositions(),
                new Pose2d(0, 0, new Rotation2d()));

        driveController.setTolerance(0.05);
        rotationController.setTolerance(0.05);
        rotationController.enableContinuousInput(0, 2 * Math.PI);

        AutoManager.getInstance().setResetOdometryConsumer(this::resetPoseEstimator);

        if (RobotBase.isSimulation()) {
            simOdometry = new SwerveDriveOdometry(KINEMATICS, getRotation(), getModulePositions(), new Pose2d());
        }

        sysIdDrive = new SysIdRoutine(
                new SysIdRoutine.Config(null, Volts.of(3), null,
                        (state) -> SignalLogger.writeString("sysid_state", state.toString())),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> {
                            drive(new ChassisSpeeds(
                                    SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND * volts.magnitude() / 12.0,
                                    0,
                                    0));
                        },
                        log -> {
                            log.motor("frontLeft")
                                    .voltage(sysidDriveAppliedVoltageMeasure
                                            .mut_replace(frontLeft.getDriveMotorVoltage(), Volts))
                                    .linearPosition(
                                            sysidDrivePositionMeasure.mut_replace(frontLeft.getDriveMotorPosition(),
                                                    Meters))
                                    .linearVelocity(sysidDriveVelocityMeasure
                                            .mut_replace(frontLeft.getDriveMotorVelocity(), MetersPerSecond));
                            log.motor("frontRight")
                                    .voltage(sysidDriveAppliedVoltageMeasure.mut_replace(
                                            frontRight.getDriveMotorVoltage(),
                                            Volts))
                                    .linearPosition(
                                            sysidDrivePositionMeasure.mut_replace(frontRight.getDriveMotorPosition(),
                                                    Meters))
                                    .linearVelocity(
                                            sysidDriveVelocityMeasure.mut_replace(frontRight.getDriveMotorVelocity(),
                                                    MetersPerSecond));
                            log.motor("backLeft")
                                    .voltage(
                                            sysidDriveAppliedVoltageMeasure.mut_replace(backLeft.getDriveMotorVoltage(),
                                                    Volts))
                                    .linearPosition(
                                            sysidDrivePositionMeasure.mut_replace(backLeft.getDriveMotorPosition(),
                                                    Meters))
                                    .linearVelocity(
                                            sysidDriveVelocityMeasure.mut_replace(backLeft.getDriveMotorVelocity(),
                                                    MetersPerSecond));
                            log.motor("backRight")
                                    .voltage(
                                            sysidDriveAppliedVoltageMeasure.mut_replace(
                                                    backRight.getDriveMotorVoltage(),
                                                    Volts))
                                    .linearPosition(
                                            sysidDrivePositionMeasure.mut_replace(backRight.getDriveMotorPosition(),
                                                    Meters))
                                    .linearVelocity(
                                            sysidDriveVelocityMeasure.mut_replace(backRight.getDriveMotorVelocity(),
                                                    MetersPerSecond));
                        },
                        this));

        sysIdSteer = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> {
                            drive(new ChassisSpeeds(
                                    SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND * volts.magnitude() /
                                            12.0,
                                    0,
                                    0));
                        },
                        log -> {
                            log.motor("frontLeft")
                                    .voltage(
                                            sysidSteerAppliedVoltageMeasure.mut_replace(
                                                    frontLeft.getSteerMotorVoltage(),
                                                    Volts))
                                    .angularPosition(
                                            sysidSteerPositionMeasure.mut_replace(frontLeft.getSteerMotorPosition(),
                                                    Rotations))
                                    .angularVelocity(
                                            sysidSteerVelocityMeasure.mut_replace(frontLeft.getSteerMotorVelocity(),
                                                    RotationsPerSecond));
                            log.motor("frontRight")
                                    .voltage(
                                            sysidSteerAppliedVoltageMeasure.mut_replace(
                                                    frontRight.getSteerMotorVoltage(),
                                                    Volts))
                                    .angularPosition(
                                            sysidSteerPositionMeasure.mut_replace(frontRight.getSteerMotorPosition(),
                                                    Rotations))
                                    .angularVelocity(
                                            sysidSteerVelocityMeasure.mut_replace(frontRight.getSteerMotorVelocity(),
                                                    RotationsPerSecond));
                            log.motor("backLeft")
                                    .voltage(
                                            sysidSteerAppliedVoltageMeasure.mut_replace(backLeft.getSteerMotorVoltage(),
                                                    Volts))
                                    .angularPosition(
                                            sysidSteerPositionMeasure.mut_replace(backLeft.getSteerMotorPosition(),
                                                    Rotations))
                                    .angularVelocity(
                                            sysidSteerVelocityMeasure.mut_replace(backLeft.getSteerMotorVelocity(),
                                                    RotationsPerSecond));
                            log.motor("backRight")
                                    .voltage(
                                            sysidSteerAppliedVoltageMeasure.mut_replace(
                                                    backRight.getSteerMotorVoltage(),
                                                    Volts))
                                    .angularPosition(
                                            sysidSteerPositionMeasure.mut_replace(backRight.getSteerMotorPosition(),
                                                    Rotations))
                                    .angularVelocity(
                                            sysidSteerVelocityMeasure.mut_replace(backRight.getSteerMotorVelocity(),
                                                    RotationsPerSecond));
                        },
                        this));

        orchestra.addInstrument(frontLeft.getDriveMotor(), 1);
        orchestra.addInstrument(frontRight.getDriveMotor(), 1);
        orchestra.addInstrument(backLeft.getDriveMotor(), 1);
        orchestra.addInstrument(backRight.getDriveMotor(), 2);

        orchestra.addInstrument(frontLeft.getSteerMotor(), 2);
        orchestra.addInstrument(backRight.getSteerMotor(), 2);
        orchestra.addInstrument(backLeft.getSteerMotor(), 3);
        orchestra.addInstrument(frontRight.getSteerMotor(), 4);
    }

    @Override
    public void periodic() {
        updatePoseEstimator();

        prevFieldRelVelocities = getFieldRelativeSpeeds();
    }

    /**
     * Updates simulation-specific variables.
     */
    @Override
    public void simulationPeriodic() {
        SwerveModulePosition[] currentPositions = getModulePositions();
        SwerveModulePosition[] deltas = new SwerveModulePosition[4];

        for (int i = 0; i < 4; i++) {
            deltas[i] = new SwerveModulePosition(
                    currentPositions[i].distanceMeters - lastModulePositions[i].distanceMeters,
                    currentPositions[i].angle);
        }

        pigeon.getSimState().setRawYaw(pigeon.getYaw().getValue() +
                Units.radiansToDegrees(KINEMATICS.toTwist2d(deltas).dtheta));

        lastModulePositions = currentPositions;
    }

    @Override
    public DriveMode getDriveMode() {
        return driveMode;
    }

    @Override
    @Log
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Log
    public Pose2d getSimPose() {
        if (simOdometry != null)
            return simOdometry.getPoseMeters();
        else
            return new Pose2d();
    }

    @Override
    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(pigeon.getYaw().getValue());
    }

    @Override
    @Log(groups = "control")
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    @Override
    @Log(groups = "control")
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
    }

    @Override
    @Log(groups = "control")
    public ChassisSpeeds getChassisSpeeds() {
        return KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    @Log(groups = "control")
    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(KINEMATICS.toChassisSpeeds(getModuleStates()), getRotation());
    }

    @Log(groups = "control")
    public ChassisAccelerations getFieldRelativeAccelerations() {
        return new ChassisAccelerations(getFieldRelativeSpeeds(), prevFieldRelVelocities, 0.020);
    }

    @Override
    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    @Override
    public void updatePoseEstimator() {
        poseEstimator.update(getRotation(), getModulePositions());
        if (RobotBase.isSimulation()) {
            simOdometry.update(getRotation(), getModulePositions());
            drawRobotOnField(AutoManager.getInstance().getField());
        }
    }

    @Override
    public void resetPoseEstimator(Pose2d pose) {
        poseEstimator.resetPosition(getRotation(), getModulePositions(), pose);
        if (RobotBase.isSimulation())
            simOdometry.resetPosition(getRotation(), getModulePositions(), pose);
    }

    @Override
    public void resetGyro() {
        pigeon.setYaw(0);
    }

    @Override
    public void setMotorHoldModes(MotorHoldMode motorHoldMode) {
        frontLeft.setMotorHoldMode(motorHoldMode);
        frontRight.setMotorHoldMode(motorHoldMode);
        backLeft.setMotorHoldMode(motorHoldMode);
        backRight.setMotorHoldMode(motorHoldMode);
    }

    @Override
    public void setDriveCurrentLimit(int currentLimit) {
        frontLeft.setDriveCurrentLimit(currentLimit);
        frontRight.setDriveCurrentLimit(currentLimit);
        backLeft.setDriveCurrentLimit(currentLimit);
        backRight.setDriveCurrentLimit(currentLimit);
    }

    @Override
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    @Override
    public void setStates(SwerveModuleState[] states) {
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }

    @Override
    public void setStatesClosedLoop(SwerveModuleState[] states) {
        frontLeft.setStateClosedLoop(states[0]);
        frontRight.setStateClosedLoop(states[1]);
        backLeft.setStateClosedLoop(states[2]);
        backRight.setStateClosedLoop(states[3]);
    }

    public void drive(ChassisSpeeds speeds) {
        drive(speeds, this.driveMode);
    }

    @Override
    public void drive(ChassisSpeeds speeds, DriveMode driveMode) {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
                && driveMode == DriveMode.FIELD_ORIENTED) {
            speeds.vxMetersPerSecond *= -1;
            speeds.vyMetersPerSecond *= -1;
        }

        commandedChassisSpeeds = speeds;
        ChassisSpeeds adjustedChassisSpeeds = null;
        switch (driveMode) {
            case ROBOT_RELATIVE:
                adjustedChassisSpeeds = speeds;
                break;
            case FIELD_ORIENTED:
                adjustedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond, getRotation());
                break;
        }

        // compensates for swerve skew
        adjustedChassisSpeeds = ChassisSpeeds.discretize(adjustedChassisSpeeds, 0.02);
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(adjustedChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states,
                SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND);

        states = new SwerveModuleState[] {
                SwerveModuleState.optimize(states[0], frontLeft.getWheelAngle()),
                SwerveModuleState.optimize(states[1], frontRight.getWheelAngle()),
                SwerveModuleState.optimize(states[2], backLeft.getWheelAngle()),
                SwerveModuleState.optimize(states[3], backRight.getWheelAngle()),
        };

        commandedModuleStates = states;
        setStates(states);
    }

    @Override
    public void driveClosedLoop(ChassisSpeeds speeds, DriveMode driveMode) {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
                && driveMode == DriveMode.FIELD_ORIENTED) {
            speeds.vxMetersPerSecond *= -1;
            speeds.vyMetersPerSecond *= -1;
        }

        commandedChassisSpeeds = speeds;
        ChassisSpeeds adjustedChassisSpeeds = null;
        switch (driveMode) {
            case ROBOT_RELATIVE:
                adjustedChassisSpeeds = speeds;
                break;
            case FIELD_ORIENTED:
                adjustedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond, getRotation());
                break;
        }

        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(adjustedChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states,
                SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND);

        states = new SwerveModuleState[] {
                SwerveModuleState.optimize(states[0], frontLeft.getWheelAngle()),
                SwerveModuleState.optimize(states[1], frontRight.getWheelAngle()),
                SwerveModuleState.optimize(states[2], backLeft.getWheelAngle()),
                SwerveModuleState.optimize(states[3], backRight.getWheelAngle()),
        };

        commandedModuleStates = states;
        setStatesClosedLoop(states);
    }

    @Override
    public Command teleopDriveCommand(DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier,
            DoubleSupplier thetaSpeedSupplier) {
        SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);
        SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);
        SlewRateLimiter thetaSpeedLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);

        return run(() -> {
            double xSpeed = xSpeedSupplier.getAsDouble();
            double ySpeed = ySpeedSupplier.getAsDouble();
            double thetaSpeed = thetaSpeedSupplier.getAsDouble();

            xSpeed = MathUtil.applyDeadband(xSpeed, JOYSTICK_INPUT_DEADBAND);
            ySpeed = MathUtil.applyDeadband(ySpeed, JOYSTICK_INPUT_DEADBAND);
            thetaSpeed = MathUtil.applyDeadband(thetaSpeed, JOYSTICK_INPUT_DEADBAND);

            xSpeed = Math.copySign(Math.pow(xSpeed, JOYSTICK_CURVE_EXP), xSpeed);
            ySpeed = Math.copySign(Math.pow(ySpeed, JOYSTICK_CURVE_EXP), ySpeed);
            thetaSpeed = Math.copySign(Math.pow(thetaSpeed, JOYSTICK_ROT_CURVE_EXP), thetaSpeed);

            xSpeed = xSpeedLimiter.calculate(xSpeed);
            ySpeed = ySpeedLimiter.calculate(ySpeed);
            thetaSpeed = thetaSpeedLimiter.calculate(thetaSpeed);

            if (isControlledRotationEnabled) {
                thetaSpeed = rotationController.calculate(getRotation().getRadians());
            }

            // the speeds are initially values from -1.0 to 1.0, so we multiply by the max
            // physical velocity to output in m/s.
            xSpeed *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
            ySpeed *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
            thetaSpeed *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND;

            driveClosedLoop(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed), driveMode);
        }).withName("drivetrain.teleopDrive");
    }

    @Override
    public Command controlledRotateCommand(DoubleSupplier angle, DriveMode driveMode) {
        return Commands.run(() -> {
            if (!isControlledRotationEnabled) {
                rotationController.reset(getRotation().getRadians());
            }
            isControlledRotationEnabled = true;
            if (driveMode == DriveMode.FIELD_ORIENTED && DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red)
                rotationController.setGoal(angle.getAsDouble() + Math.PI);
            else
                rotationController.setGoal(angle.getAsDouble());
        }).withName("drivetrain.controlledRotate");
    }

    @Override
    public Command disableControlledRotateCommand() {
        return Commands.runOnce(
                () -> {
                    isControlledRotationEnabled = false;
                }).withName("drivetrain.disableControlledRotate");
    }

    @Override
    public Command wheelLockCommand() {
        return run(() -> {
            setStates(new SwerveModuleState[] {
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45))
            });
        }).withName("drivetrain.wheelLock");
    }

    @Override
    public Command turnWheelsToAngleCommand(double angle) {
        return Commands.runOnce(() -> {
            setStates(new SwerveModuleState[] {
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle))

            });
        }).withName("drivetrain.turnWheelsToAngle");
    }

    @Override
    public Command driveToPoseCommand(Supplier<Pose2d> poseSupplier) {
        return runOnce(() -> {
            driveController.reset(getPose().getTranslation().getDistance(poseSupplier.get().getTranslation()));
            rotationController.reset(getPose().getRotation().getRadians());
        }).andThen(run(() -> {
            poseDistance = getPose().getTranslation().getDistance(poseSupplier.get().getTranslation());
            double driveVelocityScalar = driveController.getSetpoint().velocity + driveController.calculate(
                    poseDistance, 0.0);
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
                driveVelocityScalar *= -1;
            }
            if (driveController.atGoal())
                driveVelocityScalar = 0.0;

            Translation2d driveVelocity = new Pose2d(
                    new Translation2d(),
                    getPose().getTranslation().minus(poseSupplier.get().getTranslation()).getAngle())
                    .transformBy(new Transform2d(driveVelocityScalar, 0, new Rotation2d()))
                    .getTranslation();

            drive(
                    new ChassisSpeeds(
                            driveVelocity.getX(),
                            driveVelocity.getY(),
                            rotationController.calculate(getPose().getRotation().getRadians(),
                                    poseSupplier.get().getRotation().getRadians())),
                    DriveMode.FIELD_ORIENTED);
        })).withName("drivetrain.driveToPose");
    }

    @Override
    public Command followPathCommand(PathPlannerPath path) {
        return new FollowPathHolonomic(
                path,
                this::getPose,
                this::getChassisSpeeds,
                (speeds) -> driveClosedLoop(speeds, DriveMode.ROBOT_RELATIVE),
                new HolonomicPathFollowerConfig(
                        new PIDConstants(PATH_FOLLOWING_TRANSLATION_kP, 0, 0),
                        new PIDConstants(PATH_FOLLOWING_ROTATION_kP, 0, 0),
                        SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND,
                        0.3727,
                        new ReplanningConfig()),
                () -> false,
                this).finallyDo(this::stop).withName("drivetrain.followPath");
    }

    @Override
    public Command driveDeltaCommand(Transform2d delta, PathConstraints constraints) {
        return new DeferredCommand(() -> followPathCommand(
                new PathPlannerPath(
                        PathPlannerPath.bezierFromPoses(
                                getPose(), getPose().plus(delta)),
                        constraints,
                        new GoalEndState(0, delta.getRotation().plus(getRotation())))),
                Set.of()).withName("drivetrain.driveDelta");
    }

    @Override
    public Command setDriveModeCommand(DriveMode driveMode) {
        return runOnce(() -> this.driveMode = driveMode).withName("drivetrain.setDriveMode");
    }

    @Override
    public Command resetGyroCommand() {
        return runOnce(() -> resetGyro()).withName("drivetrain.resetGyro");
    }

    @Override
    public Command setDriveCurrentLimitCommand(int currentLimit) {
        return Commands.runOnce(() -> {
            frontLeft.setDriveCurrentLimit(currentLimit);
            frontRight.setDriveCurrentLimit(currentLimit);
            backLeft.setDriveCurrentLimit(currentLimit);
            backRight.setDriveCurrentLimit(currentLimit);
        }).withName("drivetrain.setDriveCurrentLimit");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(this::stop)
                .andThen(() -> setMotorHoldModes(MotorHoldMode.COAST))
                .finallyDo((d) -> setMotorHoldModes(MotorHoldMode.BRAKE))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("drivetrain.coastMotors");
    }

    /**
     * Draws the robot on a Field2d. This will include the angles of the swerve
     * modules on the outsides of the robot box in Glass.
     * 
     * @param field the field to draw the robot on (usually
     *              {@code AutoManager.getInstance().getField()})
     */
    public void drawRobotOnField(Field2d field) {
        field.setRobotPose(getPose());
        if (RobotBase.isSimulation())
            field.getObject("simPose").setPose(simOdometry.getPoseMeters());

        // Draw a pose that is based on the robot pose, but shifted by the
        // translation of the module relative to robot center,
        // then rotated around its own center by the angle of the module.
        field.getObject("modules").setPoses(
                getPose().transformBy(
                        new Transform2d(SWERVE_MODULE_LOCATIONS[0],
                                getModuleStates()[0].angle)),
                getPose().transformBy(
                        new Transform2d(SWERVE_MODULE_LOCATIONS[1],
                                getModuleStates()[1].angle)),
                getPose().transformBy(
                        new Transform2d(SWERVE_MODULE_LOCATIONS[2],
                                getModuleStates()[2].angle)),
                getPose().transformBy(
                        new Transform2d(SWERVE_MODULE_LOCATIONS[3],
                                getModuleStates()[3].angle)));
    }

    public Command sysIdDriveQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdDrive.quasistatic(direction).withName("drivetrain.sysIdDriveQuasistatic");
    }

    public Command sysIdDriveDynamic(SysIdRoutine.Direction direction) {
        return sysIdDrive.dynamic(direction).withName("drivetrain.sysIdDriveQuasistatic");
    }

    public Command sysIdSteerQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdSteer.quasistatic(direction).withName("drivetrain.sysIdDriveQuasistatic");
    }

    public Command sysIdSteerDynamic(SysIdRoutine.Direction direction) {
        return sysIdSteer.dynamic(direction).withName("drivetrain.sysIdDriveQuasistatic");
    }

    @Log
    public double getShotDistance() {
        Pose3d target = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red
                        ? Reflector.reflectPose3d(FieldConstants.SPEAKER_TARGET,
                                FieldConstants.FIELD_LENGTH)
                        : FieldConstants.SPEAKER_TARGET;

        Transform3d diff = new Pose3d(getPose()).minus(target);
        return new Translation2d(diff.getX(), diff.getY()).getNorm();
    }

    public Command targetSpeakerCommand() {
        return targetSpeakerCommand(() -> FieldConstants.SPEAKER_TARGET);
    }

    public Command targetSpeakerCommand(Supplier<Pose3d> targetPose) {
        return controlledRotateCommand(() -> {
            Pose2d target = DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red
                            ? Reflector.reflectPose2d(targetPose.get().toPose2d(),
                                    FieldConstants.FIELD_LENGTH)
                            : targetPose.get().toPose2d();
            Transform2d diff = getPose().minus(target);
            Rotation2d rot = new Rotation2d(diff.getX(), diff.getY());
            rot = rot.plus(new Rotation2d(Math.PI));
            return rot.getRadians();
        }, DriveMode.FIELD_ORIENTED);
    }

    public Command targetStageCommand(Supplier<Double> xJoystickSupplier,
            Supplier<Double> yJoystickSupplier) {
        SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);
        SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);

        ProfiledPIDController yPidController = new ProfiledPIDController(XY_kP, XY_kI, XY_kD,
                new TrapezoidProfile.Constraints(1, 1));

        return runOnce(() -> {
            List<Pose2d> stagePoses;
            Pose2d currentPose = getPose();
            if (DriverStation.getAlliance().isPresent() &&
                    DriverStation.getAlliance().get() == Alliance.Red) {
                stagePoses = List.of(
                        Reflector.reflectPose2d(FieldConstants.LEFT_CHAIN_CENTER,
                                FieldConstants.FIELD_LENGTH),
                        Reflector.reflectPose2d(FieldConstants.RIGHT_CHAIN_CENTER,
                                FieldConstants.FIELD_LENGTH),
                        Reflector.reflectPose2d(FieldConstants.FAR_CHAIN_CENTER,
                                FieldConstants.FIELD_LENGTH));
            } else {
                stagePoses = List.of(
                        FieldConstants.LEFT_CHAIN_CENTER,
                        FieldConstants.RIGHT_CHAIN_CENTER,
                        FieldConstants.FAR_CHAIN_CENTER);
            }

            targettedStagePose = currentPose.nearest(stagePoses);
        }).andThen(run(() -> {

            double cosTheta = targettedStagePose.getRotation().getCos();
            double sinTheta = targettedStagePose.getRotation().getSin();

            double A = -sinTheta;
            double B = cosTheta;
            double C = -(A * targettedStagePose.getX() + B * targettedStagePose.getY());

            // formula for distance between point and line given Ax + By + C = 0
            distance = (A * getPose().getX() + B * getPose().getY() + C) / Math.sqrt(A * A + B * B);

            double ySpeedRelStage = yPidController.calculate(distance, 0.0);

            double xJoystick = xJoystickSupplier.get();
            xJoystick = MathUtil.applyDeadband(xJoystick, JOYSTICK_INPUT_DEADBAND);
            xJoystick = Math.copySign(Math.pow(xJoystick, JOYSTICK_CURVE_EXP), xJoystick);
            xJoystick = xSpeedLimiter.calculate(xJoystick);
            xJoystick *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND / 2.0;
            if (DriverStation.getAlliance().isPresent() &&
                    DriverStation.getAlliance().get() == Alliance.Red) {
                xJoystick *= -1;
            }

            double yJoystick = yJoystickSupplier.get();
            yJoystick = MathUtil.applyDeadband(yJoystick, JOYSTICK_INPUT_DEADBAND);
            yJoystick = Math.copySign(Math.pow(yJoystick, JOYSTICK_CURVE_EXP), yJoystick);
            yJoystick = ySpeedLimiter.calculate(yJoystick);
            yJoystick *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND / 2.0;

            double lineDirX = Math.cos(targettedStagePose.getRotation().getRadians());
            double lineDirY = Math.sin(targettedStagePose.getRotation().getRadians());

            double xSpeedRelStage = xJoystick * lineDirX + yJoystick * lineDirY;

            double xSpeed = xSpeedRelStage * cosTheta - ySpeedRelStage * sinTheta;
            double ySpeed = xSpeedRelStage * sinTheta + ySpeedRelStage * cosTheta;

            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
                xSpeed *= -1;
                ySpeed *= -1;
            }

            double thetaSpeed = rotationController.calculate(getRotation().getRadians(),
                    targettedStagePose.getRotation().getRadians());

            drive(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed), DriveMode.FIELD_ORIENTED);
        })).withName("drivetrain.teleopDrive");
    }

    public Command playMusicCommand(MusicTrack track) {
        return startEnd(() -> {
            orchestra.loadMusic(track.filename);
            orchestra.play();
        }, orchestra::pause);
    }

    public Pose3d calculateEffectiveTargetLocation() {
        return TrajectoryCalcs.calculateEffectiveTargetLocation(getPose(), getFieldRelativeSpeeds(),
                getFieldRelativeAccelerations());
    }

}
