package frc.robot.subsystems;

import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Constants.Teleop.*;

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
import com.techhounds.houndutil.houndlib.MotorHoldMode;
import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive;
import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive.DriveMode;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@LoggedObject
public class Drivetrain extends SubsystemBase implements BaseSwerveDrive {

    @Log(groups = "modules")
    private KrakenCoaxialSwerveModule frontLeft = new KrakenCoaxialSwerveModule(FRONT_LEFT_DRIVE_MOTOR_ID,
            FRONT_LEFT_STEER_MOTOR_ID,
            FRONT_LEFT_STEER_ENCODER_ID, CAN_BUS, FRONT_LEFT_DRIVE_INVERTED, FRONT_LEFT_STEER_INVERTED,
            FRONT_LEFT_CANCODER_INVERTED,
            FRONT_LEFT_OFFSET,
            SWERVE_CONSTANTS); // Inverts Untested

    @Log(groups = "modules")
    private KrakenCoaxialSwerveModule frontRight = new KrakenCoaxialSwerveModule(FRONT_RIGHT_DRIVE_MOTOR_ID,
            FRONT_RIGHT_STEER_MOTOR_ID,
            FRONT_RIGHT_STEER_ENCODER_ID, CAN_BUS, FRONT_RIGHT_DRIVE_INVERTED, FRONT_RIGHT_STEER_INVERTED,
            FRONT_RIGHT_CANCODER_INVERTED,
            FRONT_RIGHT_OFFSET,
            SWERVE_CONSTANTS); // Inverts Untested

    @Log(groups = "modules")
    private KrakenCoaxialSwerveModule backLeft = new KrakenCoaxialSwerveModule(BACK_LEFT_DRIVE_MOTOR_ID,
            BACK_LEFT_STEER_MOTOR_ID,
            BACK_LEFT_STEER_ENCODER_ID, CAN_BUS, BACK_LEFT_DRIVE_INVERTED, BACK_LEFT_STEER_INVERTED,
            BACK_LEFT_CANCODER_INVERTED,
            BACK_LEFT_OFFSET,
            SWERVE_CONSTANTS); // Inverts Untested

    @Log(groups = "moudules")
    private KrakenCoaxialSwerveModule backRight = new KrakenCoaxialSwerveModule(BACK_RIGHT_DRIVE_MOTOR_ID,
            BACK_RIGHT_STEER_MOTOR_ID,
            BACK_RIGHT_STEER_ENCODER_ID, CAN_BUS, BACK_RIGHT_DRIVE_INVERTED, BACK_RIGHT_STEER_INVERTED,
            BACK_RIGHT_CANCODER_INVERTED,
            BACK_RIGHT_OFFSET,
            SWERVE_CONSTANTS); // Inverts Untested

    /**
     * The controller that allows the drivetrain to maintain or turn to a specific
     * angle
     */
    @Log(name = "Rotation Controller")
    private ProfiledPIDController rotationController = new ProfiledPIDController(SWERVE_CONSTANTS.STEER_kP,
            SWERVE_CONSTANTS.STEER_kI,
            SWERVE_CONSTANTS.STEER_kD,
            new TrapezoidProfile.Constraints(20 * Math.PI,
                    20 * Math.PI));

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

    /**
     * Whether to override the inputs of the driver for maintaining or turning to a
     * specific angle.
     */
    @Log
    private boolean isControlledRotationEnabled = false;

    @Log(groups = "control")
    private ChassisSpeeds commandedChassisSpeeds = new ChassisSpeeds();

    // initiate pigeon gyro -- Jake
    @Log
    private Pigeon2 gyro = new Pigeon2(GYRO_DEVICE_ID);

    @Log
    private SwerveDrivePoseEstimator poseEstimator;

    @Log
    private DriveMode driveMode = DriveMode.FIELD_ORIENTED;

    public Drivetrain() {
        resetGyro();

        poseEstimator = new SwerveDrivePoseEstimator(KINEMATICS, getRotation(), getModulePositions(), new Pose2d());

        thetaPositionController.setTolerance(0.05);
        thetaPositionController.enableContinuousInput(0, 2 * Math.PI);
    }

    /**
     * Gets current drive mode.
     */
    @Override
    public DriveMode getDriveMode() {
        return driveMode;
    }

    /**
     * Gets current pose.
     */
    @Override
    @Log
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Gets the current heading of the gyro.
     */
    @Override
    public Rotation2d getRotation() {
        return gyro.getRotation2d();
    }

    /**
     * Gets the current swerve module positions.
     */
    @Override
    @Log(groups = "control")
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                backLeft.getPosition(),
                frontRight.getPosition(),
                backRight.getPosition()
        };
    }

    /**
     * Gets the current swerve module states (encoder velocities).
     */
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

    /*
     * Gets current chassis speed after calculating with kinematics.
     */
    @Override
    @Log(groups = "control")
    public ChassisSpeeds getChassisSpeeds() {
        return KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    /**
     * Gets pose estimator.
     */
    @Override
    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    /** Update the pose estimator. */
    @Override
    public void updatePoseEstimator() {
        poseEstimator.update(getRotation(), getModulePositions());
    }

    /** ReSets the pose estimator. */
    @Override
    public void resetPoseEstimator(Pose2d pose) {
        poseEstimator.resetPosition(getRotation(), getModulePositions(), pose);
    }

    /** Resets gyro. */
    @Override
    public void resetGyro() {
        gyro.reset();
    }

    /** Sets the serve drive motor hold modes. */
    @Override
    public void setMotorHoldModes(MotorHoldMode motorHoldMode) {
        frontLeft.setMotorHoldMode(motorHoldMode);
        frontRight.setMotorHoldMode(motorHoldMode);
        backLeft.setMotorHoldMode(motorHoldMode);
        backRight.setMotorHoldMode(motorHoldMode);
    }

    /**
     * Sets the swerve module current limits to the argument supplied.
     */
    @Override
    public void setDriveCurrentLimit(int currentLimit) {
        frontLeft.setDriveCurrentLimit(currentLimit);
        frontRight.setDriveCurrentLimit(currentLimit);
        backLeft.setDriveCurrentLimit(currentLimit);
        backRight.setDriveCurrentLimit(currentLimit);
    }

    /** stop all swerve drives. */
    @Override
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Sets states for the swerve modules.
     */
    @Override
    public void setStates(SwerveModuleState[] state) {
        frontLeft.setState(state[0]);
        frontRight.setState(state[1]);
        backLeft.setState(state[2]);
        backRight.setState(state[3]);
    }

    /**
     * Sets closed loop states for the swerve modules.
     */
    @Override
    public void setStatesClosedLoop(SwerveModuleState[] state) {
        frontLeft.setStateClosedLoop(state[0]);
        frontRight.setStateClosedLoop(state[1]);
        backLeft.setStateClosedLoop(state[2]);
        backRight.setStateClosedLoop(state[3]);
    }

    /**
     * Drives at current drivemode.
     */

    public void drive(ChassisSpeeds speeds) {
        drive(speeds, this.driveMode);
    }

    /**
     * Drives with inputted drivemode.
     */
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

    /**
     * Drives at current drivemode with closed loop controls.
     */
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

    /**
     * Drives with teleop.
     */
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

            xSpeed = MathUtil.applyDeadband(xSpeed, 0); // Untested
            ySpeed = MathUtil.applyDeadband(ySpeed, 0); // Untested
            thetaSpeed = MathUtil.applyDeadband(thetaSpeed, 0); // Untested

            xSpeed = Math.copySign(Math.pow(xSpeed, JOYSTICK_CURVE_EXP), xSpeed);
            ySpeed = Math.copySign(Math.pow(ySpeed, JOYSTICK_CURVE_EXP), ySpeed);
            thetaSpeed = Math.copySign(Math.pow(thetaSpeed, JOYSTICK_CURVE_EXP), thetaSpeed);

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

        }).withName("Teleop Drive Command");
    }

    /**
     * Creates a command that allows for rotation to any angle.
     * 
     * Map this to a button input to rotate while still translating.
     */
    @Override
    public Command disableControlledRotateCommand() {
        return runOnce(() -> {
            isControlledRotationEnabled = false;
        });
    }

    /**
     * Creates the command to lock all wheels.
     */
    @Override
    public Command wheelLockCommand() {
        return run(() -> {
            setStates(new SwerveModuleState[] {
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45))
            });
        }).withName("Wheel Lock");
    }

    /*
     * Creates the command to make all wheels turn to an angle.
     */
    @Override
    public Command turnWheelsToAngleCommand(double angle) {
        return runOnce(() -> {
            setStates(new SwerveModuleState[] {
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle))

            });
        });
    }

    /**
     * Drives to inputted pose.
     */
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
        }));
    }

    /**
     * Drives by following the command.
     */
    @Override
    public Command followPathCommand(PathPlannerPath path) {
        return new FollowPathHolonomic(path, this::getPose, this::getChassisSpeeds,
                (speeds) -> driveClosedLoop(speeds, DriveMode.ROBOT_RELATIVE),
                new HolonomicPathFollowerConfig(
                        new PIDConstants(PATH_FOLLOWING_TRANSLATION_kP, 0, 0),
                        new PIDConstants(PATH_FOLLOWING_ROTATION_kP, 0, 0),
                        SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND,
                        0.41,
                        new ReplanningConfig()),
                () -> (driveMode == DriveMode.FIELD_ORIENTED
                        && DriverStation.getAlliance().get() == Alliance.Red),
                this);
    }

    /**
     * Drive based on the delta (change).
     */
    @Override
    public Command driveDeltaCommand(Transform2d delta, PathConstraints constraints) {
        return new DeferredCommand(() -> followPathCommand(
                new PathPlannerPath(
                        PathPlannerPath.bezierFromPoses(
                                getPose(), getPose().plus(delta)),
                        constraints,
                        new GoalEndState(0, delta.getRotation().plus(getRotation())))),
                Set.of());
    }

    /*
     * Sets the drive mode as a command.
     */
    @Override
    public Command setDriveModeCommand(DriveMode driveMode) {
        return runOnce(() -> this.driveMode = driveMode);
    }

    /*
     * resets the gyro command.
     */
    @Override
    public Command resetGyroCommand() {
        return runOnce(() -> {
            gyro.reset();
        });

    }

    /*
     * Creates a command that sets the drive current limits for each swerve modules.
     */
    @Override
    public Command setDriveCurrentLimitCommand(int currentLimit) {
        return runOnce(() -> {
            frontLeft.setDriveCurrentLimit(currentLimit);
            frontRight.setDriveCurrentLimit(currentLimit);
            backLeft.setDriveCurrentLimit(currentLimit);
            backRight.setDriveCurrentLimit(currentLimit);
        });
    }

    /*
     * Creates a command that sets motors to coast for the duration of the command.
     */
    @Override
    public Command coastMotorsCommand() {
        return runOnce(this::stop)
                .andThen(() -> setMotorHoldModes(MotorHoldMode.COAST))
                .finallyDo((d) -> setMotorHoldModes(MotorHoldMode.BRAKE))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    /*
     * Creates a command to rotate in a controlled manner to the target angle.
     */
    @Override
    public Command controlledRotateCommand(DoubleSupplier angle, DriveMode driveMode) {
        return startEnd(() -> {
            if (!isControlledRotationEnabled) {
                rotationController.reset((getRotation()).getRadians());
            }
            isControlledRotationEnabled = true;
            if (driveMode == DriveMode.FIELD_ORIENTED && DriverStation.getAlliance().get() == Alliance.Red) {
                rotationController.setGoal(angle.getAsDouble() + Math.PI);
            } else {
                rotationController.setGoal(angle.getAsDouble());
            }
        }, () -> {
            isControlledRotationEnabled = false;
        }).withName("Turn While Moving");
    }
}
