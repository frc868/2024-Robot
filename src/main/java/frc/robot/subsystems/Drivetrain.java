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

public class Drivetrain implements BaseSwerveDrive {
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

    private Pigeon2 gyro = new Pigeon2(0);

    private SwerveDrivePoseEstimator poseEstimator;

    private DriveMode driveMode = DriveMode.FIELD_ORIENTED;

    public Drivetrain() {
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    public Pose2d getPose() {
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
                frontLeft.getState(),
                backLeft.getState(),
                frontRight.getState(),
                backRight.getState()
        };
    }

    public ChassisSpeeds getChassisSpeeds() {
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
    }

    public void updatePoseEstimator() {
    }

    public void resetPoseEstimator(Pose2d pose) {
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void setMotorHoldModes(MotorHoldMode motorHoldMode) {
    }

    public void setDriveCurrentLimit(int currentLimit) {
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setStates(SwerveModuleState[] state) {
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

    public Command turnWheelsToAngleCommand(double angle) {
    }

    public Command driveToPoseCommand(Pose2d pose) {
    }

    public Command followPathCommand(PathPlannerPath path) {
    }

    public Command driveDeltaCommand(Transform2d delta, PathConstraints constraints) {
    }

    public Command setDriveModeCommand(DriveMode driveMode) {
    }

    public Command resetGyroCommand() {

    }

    public Command setDriveCurrentLimitCommand(int currentLimit) {
    }

    public Command coastMotorsCommand() {
    }
}
