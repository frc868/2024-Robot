package frc.robot.subsystems;

import java.util.Set;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.techhounds.houndutil.houndauto.AutoManager;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    private Pigeon2 pigeon = new Pigeon2(0);

    @Log
    private SwerveDrivePoseEstimator poseEstimator;

    @Log
    private ProfiledPIDController xPositionController = new ProfiledPIDController(
            XY_kP, XY_kI, XY_kD, XY_CONSTRAINTS);

    @Log
    private ProfiledPIDController yPositionController = new ProfiledPIDController(
            XY_kP, XY_kI, XY_kD, XY_CONSTRAINTS);

    @Log
    private ProfiledPIDController thetaPositionController = new ProfiledPIDController(
            THETA_kP, THETA_kI, THETA_kD, THETA_CONSTRAINTS);

    @Log(groups = "control")
    private SwerveModuleState[] commandedModuleStates = new SwerveModuleState[] { new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };

    @Log(groups = "control")
    private ChassisSpeeds commandedChassisSpeeds = new ChassisSpeeds();

    @Log
    private DriveMode driveMode = DriveMode.FIELD_ORIENTED;

    @Log
    private Twist2d twist = new Twist2d();

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

    private final SysIdRoutine sysIdDrive = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> {
                        drive(new ChassisSpeeds(
                                SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND * volts.magnitude() / 12.0, 0,
                                0));
                    },
                    log -> {
                        log.motor("frontLeft")
                                .voltage(
                                        sysidDriveAppliedVoltageMeasure.mut_replace(frontLeft.getDriveMotorVoltage(),
                                                Volts))
                                .linearPosition(
                                        sysidDrivePositionMeasure.mut_replace(frontLeft.getDriveMotorPosition(),
                                                Meters))
                                .linearVelocity(sysidDriveVelocityMeasure.mut_replace(frontLeft.getDriveMotorVelocity(),
                                        MetersPerSecond));
                        log.motor("frontRight")
                                .voltage(sysidDriveAppliedVoltageMeasure.mut_replace(frontRight.getDriveMotorVoltage(),
                                        Volts))
                                .linearPosition(
                                        sysidDrivePositionMeasure.mut_replace(frontRight.getDriveMotorPosition(),
                                                Meters))
                                .linearVelocity(
                                        sysidDriveVelocityMeasure.mut_replace(frontRight.getDriveMotorVelocity(),
                                                MetersPerSecond));
                        log.motor("backLeft")
                                .voltage(sysidDriveAppliedVoltageMeasure.mut_replace(backLeft.getDriveMotorVoltage(),
                                        Volts))
                                .linearPosition(
                                        sysidDrivePositionMeasure.mut_replace(backLeft.getDriveMotorPosition(), Meters))
                                .linearVelocity(sysidDriveVelocityMeasure.mut_replace(backLeft.getDriveMotorVelocity(),
                                        MetersPerSecond));
                        log.motor("backRight")
                                .voltage(
                                        sysidDriveAppliedVoltageMeasure.mut_replace(backRight.getDriveMotorVoltage(),
                                                Volts))
                                .linearPosition(
                                        sysidDrivePositionMeasure.mut_replace(backRight.getDriveMotorPosition(),
                                                Meters))
                                .linearVelocity(sysidDriveVelocityMeasure.mut_replace(backRight.getDriveMotorVelocity(),
                                        MetersPerSecond));
                    },
                    this));

    private final MutableMeasure<Voltage> sysidSteerAppliedVoltageMeasure = mutable(Volts.of(0));
    private final MutableMeasure<Angle> sysidSteerPositionMeasure = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> sysidSteerVelocityMeasure = mutable(RotationsPerSecond.of(0));

    private final SysIdRoutine sysIdSteer = new SysIdRoutine(
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
                                        sysidSteerAppliedVoltageMeasure.mut_replace(frontLeft.getSteerMotorVoltage(),
                                                Volts))
                                .angularPosition(
                                        sysidSteerPositionMeasure.mut_replace(frontLeft.getSteerMotorPosition(),
                                                Rotations))
                                .angularVelocity(
                                        sysidSteerVelocityMeasure.mut_replace(frontLeft.getSteerMotorVelocity(),
                                                RotationsPerSecond));
                        log.motor("frontRight")
                                .voltage(
                                        sysidSteerAppliedVoltageMeasure.mut_replace(frontRight.getSteerMotorVoltage(),
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
                                        sysidSteerAppliedVoltageMeasure.mut_replace(backRight.getSteerMotorVoltage(),
                                                Volts))
                                .angularPosition(
                                        sysidSteerPositionMeasure.mut_replace(backRight.getSteerMotorPosition(),
                                                Rotations))
                                .angularVelocity(
                                        sysidSteerVelocityMeasure.mut_replace(backRight.getSteerMotorVelocity(),
                                                RotationsPerSecond));
                    },
                    this));

    /** Initializes the drivetrain. */
    public Drivetrain() {
        resetGyro();
        poseEstimator = new SwerveDrivePoseEstimator(
                KINEMATICS,
                getRotation(),
                getModulePositions(),
                new Pose2d(0, 0, new Rotation2d()));

        thetaPositionController.setTolerance(0.05);
        thetaPositionController.enableContinuousInput(0, 2 * Math.PI);

        AutoManager.getInstance().setResetOdometryConsumer(this::resetPoseEstimator);

        if (RobotBase.isSimulation()) {
            simOdometry = new SwerveDriveOdometry(KINEMATICS, getRotation(), getModulePositions(), new Pose2d());
        }
    }

    /**
     * Runs every 20ms. Do not run anything but odometry updating and debug code
     * here.
     */
    @Override
    public void periodic() {
        updatePoseEstimator();
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

        twist = KINEMATICS.toTwist2d(deltas);

        pigeon.getSimState().setRawYaw(pigeon.getYaw().getValue() +
                Units.radiansToDegrees(twist.dtheta));

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
        return simOdometry.getPoseMeters();
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

    @Override
    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    @Override
    public void updatePoseEstimator() {
        poseEstimator.update(getRotation(), getModulePositions());
        simOdometry.update(getRotation(), getModulePositions());
        drawRobotOnField(AutoManager.getInstance().getField());
    }

    @Override
    public void resetPoseEstimator(Pose2d pose) {
        poseEstimator.resetPosition(getRotation(), getModulePositions(), pose);
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
            thetaSpeed = Math.copySign(Math.pow(thetaSpeed, JOYSTICK_CURVE_EXP), thetaSpeed);

            xSpeed = xSpeedLimiter.calculate(xSpeed);
            ySpeed = ySpeedLimiter.calculate(ySpeed);
            thetaSpeed = thetaSpeedLimiter.calculate(thetaSpeed);

            if (isControlledRotationEnabled) {
                thetaSpeed = thetaPositionController.calculate(getRotation().getRadians());
            }

            // the speeds are initially values from -1.0 to 1.0, so we multiply by the max
            // physical velocity to output in m/s.
            xSpeed *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
            ySpeed *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
            thetaSpeed *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND;

            drive(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed), driveMode);
        }).withName("drivetrain.teleopDrive");
    }

    @Override
    public Command controlledRotateCommand(DoubleSupplier angle, DriveMode driveMode) {
        return Commands.run(() -> {
            if (!isControlledRotationEnabled) {
                thetaPositionController.reset(getRotation().getRadians());
            }
            isControlledRotationEnabled = true;
            if (driveMode == DriveMode.FIELD_ORIENTED && DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red)
                thetaPositionController.setGoal(angle.getAsDouble() + Math.PI);
            else
                thetaPositionController.setGoal(angle.getAsDouble());
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
    public Command driveToPoseCommand(Pose2d pose) {
        return runOnce(() -> {
            xPositionController.reset(getPose().getX());
            yPositionController.reset(getPose().getY());
            thetaPositionController.reset(getPose().getRotation().getRadians());
        }).andThen(run(() -> {
            driveClosedLoop(
                    new ChassisSpeeds(
                            xPositionController.calculate(getPose().getX(), pose.getX()),
                            yPositionController.calculate(getPose().getX(), pose.getY()),
                            thetaPositionController.calculate(getPose().getX(), pose.getRotation().getRadians())),
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
                        0.41,
                        new ReplanningConfig()),
                () -> false,
                this).withName("drivetrain.followPath");
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
        field.getObject("simPose").setPose(simOdometry.getPoseMeters());

        // Draw a pose that is based on the robot pose, but shifted by the
        // translation of the module relative to robot center,
        // then rotated around its own center by the angle of the module.
        field.getObject("frontLeft").setPose(
                getPose().transformBy(
                        new Transform2d(SWERVE_MODULE_LOCATIONS[0],
                                getModuleStates()[0].angle)));
        field.getObject("frontRight").setPose(
                getPose().transformBy(
                        new Transform2d(SWERVE_MODULE_LOCATIONS[1],
                                getModuleStates()[1].angle)));
        field.getObject("backLeft").setPose(
                getPose().transformBy(
                        new Transform2d(SWERVE_MODULE_LOCATIONS[2],
                                getModuleStates()[2].angle)));
        field.getObject("backRight").setPose(
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
}
