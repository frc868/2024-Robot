package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.techhounds.houndutil.houndlib.MotorHoldMode;
import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive;
import com.techhounds.houndutil.houndlib.swerve.KrakenCoaxialSwerveModule;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase implements BaseSwerveDrive {
    private KrakenCoaxialSwerveModule frontLeft = new KrakenCoaxialSwerveModule(0, 0, 0, null, false, false, false,
            0,
            null);

    private KrakenCoaxialSwerveModule frontRight = new KrakenCoaxialSwerveModule(0, 0, 0, null, false, false, false,
            0,
            null);


    private KrakenCoaxialSwerveModule backLeft = new KrakenCoaxialSwerveModule(0, 0, 0, null, false, false, false,
            0,
            null);

    private KrakenCoaxialSwerveModule backRight = new KrakenCoaxialSwerveModule(0, 0, 0, null, false, false, false,
            0,
            null);

    //initiate pigeon gyro -- Jake
    private Pigeon2 gyro = new Pigeon2(0);

    private SwerveDrivePoseEstimator poseEstimator;

    private DriveMode driveMode = DriveMode.FIELD_ORIENTED;

    public Drivetrain() {
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Gets the current heading of the gyro
     * 
     * @return the heading of the robot as a Rotation2d object
     */
    public Rotation2d getRotation() {
        return gyro.getRotation2d();
    }

    /**
     * Gets the current swerve module positions
     * 
     * @return the current swerve module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                backLeft.getPosition(),
                frontRight.getPosition(),
                backRight.getPosition()
        };
    }

    /**
     * Get the current swerve module states (encoder velocities).
     * 
     * @return the current swerve module states
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                frontLeft.getState(states[0]);
                frontRight.getState(states[1]);
                backLeft.getState(states[2]);
                backRight.getState(states[3]);
        };
    }

    public ChassisSpeeds getChassisSpeeds() {
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public void updatePoseEstimator() {
        poseEstimator.update(getRotation(), getModulePositions());
    }

    public void resetPoseEstimator(Pose2d pose) {
        poseEstimator.resetPosition(getRotation(), getModulePositions(), pose);
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void setMotorHoldModes(MotorHoldMode motorHoldMode) {
    }

    /**
     * Set the swerve module current limits to the argument supplied.
     * 
     */
    public void setDriveCurrentLimit(int currentLimit) {
        frontLeft.setDriveCurrentLimit(currentLimit);
        frontRight.setDriveCurrentLimit(currentLimit);
        backLeft.setDriveCurrentLimit(currentLimit);
        backRight.setDriveCurrentLimit(currentLimit);
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /*
     * Set states for the swerve modules
     */
    public void setStates(SwerveModuleState[] state) {
        frontLeft.setState(state[0]);
        frontRight.setState(state[1]);
        backLeft.setState(state[2]);
        backRight.setState(state[3]);
    }

    public void setStatesClosedLoop(SwerveModuleState[] state) {
    }

    public void drive(ChassisSpeeds speeds) {
    }

    public void drive(ChassisSpeeds speeds, DriveMode driveMode) {
    }

    public void driveClosedLoop(ChassisSpeeds speeds, DriveMode driveMode) {
    }

    public Command teleopDriveCommand(DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier,
            DoubleSupplier thetaSpeedSupplier) {
    }

    public Command controlledRotateCommand(double angle, DriveMode driveMode) {
    }

    public Command disableControlledRotateCommand() {
    }

    public Command wheelLockCommand() {
    }

    /* 
     * Creates the command to make all wheels turn to an angle
    */
    public Command turnWheelsToAngleCommand(double angle) {
        return runOnce(()->{
                setStates(new SwerveModuleState[] {
                        new SwerveModuleState(0, new Rotation2d(angle)),
                        new SwerveModuleState(0, new Rotation2d(angle)),
                        new SwerveModuleState(0, new Rotation2d(angle)),
                        new SwerveModuleState(0, new Rotation2d(angle))
                
                });
        })
    }

    public Command driveToPoseCommand(Pose2d pose) {
    }

    public Command followPathCommand(PathPlannerPath path) {
    }

    public Command driveDeltaCommand(Transform2d delta, PathConstraints constraints) {
    }

    /*
     * Sets the drive mode as a command
     */
    public Command setDriveModeCommand(DriveMode driveMode) {
        return runOnce(() -> this.driveMode = driveMode);
    }

    /*
     * resets the  gyro command                                                        
     */
    public Command resetGyroCommand() {
        return runOnce(()->{
                gyro.reset();
        });

    }

    /*
     * Sets the drive current limits for each swerve modules
     */
    public Command setDriveCurrentLimitCommand(int currentLimit) {
        return runOnce(() -> {
            frontLeft.setDriveCurrentLimit(currentLimit);
            frontRight.setDriveCurrentLimit(currentLimit);
            backLeft.setDriveCurrentLimit(currentLimit);
            backRight.setDriveCurrentLimit(currentLimit);
        });
    }

    public Command coastMotorsCommand() {
    }
}
