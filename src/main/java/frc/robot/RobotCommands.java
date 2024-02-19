package frc.robot;

import static frc.robot.Constants.Shooter.SHOOTING_RPS;

import java.lang.reflect.Field;
import java.sql.Driver;

import com.techhounds.houndutil.houndauto.Reflector;
import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive.DriveMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Climber.ClimberPosition;
import frc.robot.Constants.Intake.IntakePosition;
import frc.robot.Constants.NoteLift.NoteLiftPosition;
import frc.robot.Constants.ShooterTilt.ShooterTiltPosition;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteLift;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTilt;

public class RobotCommands {
    public static Command targetSpeakerCommand(Drivetrain drivetrain, Shooter shooter, ShooterTilt shooterTilt) {
        return Commands.parallel(
                drivetrain.controlledRotateCommand(() -> {
                    Pose2d target = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red
                                    ? Reflector.reflectPose2d(FieldConstants.TARGET.toPose2d(),
                                            FieldConstants.FIELD_LENGTH)
                                    : FieldConstants.TARGET.toPose2d();
                    Transform2d diff = drivetrain.getPose().minus(target);
                    Rotation2d rot = new Rotation2d(diff.getX(), diff.getY());
                    rot = rot.plus(new Rotation2d(Math.PI));
                    return rot.getRadians();
                }, DriveMode.FIELD_ORIENTED),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose),
                shooter.targetSpeakerCommand(drivetrain::getPose));
    }

    public static Command targetFromSubwooferCommand(Drivetrain drivetrain, Shooter shooter, ShooterTilt shooterTilt) {
        return Commands.parallel(
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.SUBWOOFER),
                shooter.spinAtVelocityCommand(() -> SHOOTING_RPS));
    }

    public static Command shootCommand(Drivetrain drivetrain, Intake intake, Shooter shooter, ShooterTilt shooterTilt) {
        return Commands.sequence(
                Commands.parallel(
                        shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                        shooter.targetSpeakerCommand(drivetrain::getPose).asProxy())
                        .until(() -> shooter.atGoal() && shooterTilt.atGoal()),
                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy().alongWith(intake.runRollersCommand())
                        .withTimeout(1));
    }

    public static Command intakeToNoteLift(Shooter shooter, ShooterTilt shooterTilt, NoteLift noteLift) {
        return Commands.parallel(
                Commands.sequence(
                        shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB).asProxy(),
                        noteLift.moveToPositionCommand(() -> NoteLiftPosition.INTAKE).asProxy()),
                shooter.stopCommand());
    }

    public static Command prepareClimb(Intake intake, Shooter shooter, ShooterTilt shooterTilt, Climber climber,
            NoteLift noteLift) {
        return Commands.parallel(
                Commands.sequence(
                        shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB).asProxy(),
                        intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy(),
                        noteLift.moveToPositionCommand(() -> NoteLiftPosition.CLIMB_PREP).asProxy(),
                        climber.moveToPositionCommand(() -> ClimberPosition.CLIMB_PREP).asProxy()),
                shooter.stopCommand());
    }

    public static Command resetClimb(Intake intake, Shooter shooter, ShooterTilt shooterTilt, Climber climber,
            NoteLift noteLift) {
        return Commands.parallel(
                Commands.sequence(
                        shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB).asProxy(),
                        climber.moveToPositionCommand(() -> ClimberPosition.BOTTOM).asProxy()));
    }
}
