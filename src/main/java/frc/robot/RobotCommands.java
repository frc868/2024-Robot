package frc.robot;

import static frc.robot.Constants.Shooter.BASE_SHOOTING_RPS;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
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
import frc.robot.utils.TrajectoryCalcs;

public class RobotCommands {
    public static Command targetSpeakerCommand(Drivetrain drivetrain, Shooter shooter, ShooterTilt shooterTilt) {
        return Commands.parallel(
                drivetrain.targetSpeakerCommand(),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose),
                shooter.targetSpeakerCommand(drivetrain::getPose)).withName("RobotCommands.targetSpeaker");
    }

    public static Command targetSpeakerOnTheMoveCommand(Drivetrain drivetrain, Shooter shooter,
            ShooterTilt shooterTilt) {
        return Commands.parallel(
                drivetrain.targetSpeakerCommand(
                        () -> TrajectoryCalcs.calculateEffectiveTargetLocation(drivetrain.getPose(),
                                drivetrain.getFieldRelativeSpeeds(), drivetrain.getFieldRelativeAccelerations())),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose,
                        () -> TrajectoryCalcs.calculateEffectiveTargetLocation(drivetrain.getPose(),
                                drivetrain.getFieldRelativeSpeeds(), drivetrain.getFieldRelativeAccelerations())),
                shooter.targetSpeakerCommand(drivetrain::getPose,
                        () -> TrajectoryCalcs.calculateEffectiveTargetLocation(drivetrain.getPose(),
                                drivetrain.getFieldRelativeSpeeds(), drivetrain.getFieldRelativeAccelerations())))
                .withName("RobotCommands.targetSpeakerOnTheMove");
    }

    public static Command targetFromSubwooferCommand(Drivetrain drivetrain, Shooter shooter, ShooterTilt shooterTilt) {
        return Commands.parallel(
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.SUBWOOFER),
                shooter.spinAtVelocityCommand(() -> BASE_SHOOTING_RPS));
    }

    public static Command shootCommand(Drivetrain drivetrain, Intake intake, Shooter shooter, ShooterTilt shooterTilt) {
        return Commands.parallel(
                // drivetrain.targetSpeakerCommand(),
                Commands.sequence(
                        Commands.parallel(
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy())
                                .until(() -> shooter.atGoal() && shooterTilt.atGoal()),
                        shooter.targetSpeakerCommand(drivetrain::getPose).asProxy()
                                .alongWith(intake.runRollersCommand())
                                .withTimeout(1)));
    }

    public static Command shootOnTheMoveCommand(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt) {
        return Commands.parallel(
                drivetrain.targetSpeakerCommand(
                        () -> TrajectoryCalcs.calculateEffectiveTargetLocation(drivetrain.getPose(),
                                drivetrain.getFieldRelativeSpeeds(), drivetrain.getFieldRelativeAccelerations())),
                Commands.sequence(
                        Commands.parallel(
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose,
                                        () -> TrajectoryCalcs.calculateEffectiveTargetLocation(drivetrain.getPose(),
                                                drivetrain.getFieldRelativeSpeeds(),
                                                drivetrain.getFieldRelativeAccelerations()))
                                        .asProxy(),
                                shooter.targetSpeakerCommand(drivetrain::getPose,
                                        () -> TrajectoryCalcs.calculateEffectiveTargetLocation(drivetrain.getPose(),
                                                drivetrain.getFieldRelativeSpeeds(),
                                                drivetrain.getFieldRelativeAccelerations()))
                                        .asProxy())
                                .until(() -> shooter.atGoal() && shooterTilt.atGoal()),
                        shooter.targetSpeakerCommand(drivetrain::getPose,
                                () -> TrajectoryCalcs.calculateEffectiveTargetLocation(drivetrain.getPose(),
                                        drivetrain.getFieldRelativeSpeeds(),
                                        drivetrain.getFieldRelativeAccelerations()))
                                .asProxy()
                                .alongWith(intake.runRollersCommand())
                                .withTimeout(1)));
    }

    public static Command intakeToNoteLift(Shooter shooter, ShooterTilt shooterTilt, NoteLift noteLift) {
        return Commands.sequence(
                new ScheduleCommand(shooter.stopCommand()),
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB).asProxy(),
                noteLift.moveToPositionCommand(() -> NoteLiftPosition.INTAKE).asProxy());
    }

    public static Command prepareClimb(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier,
            Drivetrain drivetrain, Intake intake, Shooter shooter, ShooterTilt shooterTilt, Climber climber,
            NoteLift noteLift) {
        return Commands.parallel(
                new ScheduleCommand(shooter.stopCommand()),
                drivetrain.targetStageCommand(xSpeedSupplier, ySpeedSupplier),
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB).asProxy(),
                intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy(),
                noteLift.moveToPositionCommand(() -> NoteLiftPosition.CLIMB_PREP).asProxy(),
                climber.moveToPositionCommand(() -> ClimberPosition.CLIMB_PREP).asProxy());
    }

    public static Command resetClimb(Intake intake, Shooter shooter, ShooterTilt shooterTilt, Climber climber,
            NoteLift noteLift) {
        return Commands.parallel(
                Commands.sequence(
                        shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB).asProxy(),
                        climber.moveToPositionCommand(() -> ClimberPosition.BOTTOM).asProxy()));
    }

    public static Command deClimb(Intake intake, Shooter shooter, ShooterTilt shooterTilt, Climber climber,
            NoteLift noteLift) {
        return Commands.parallel(
                Commands.sequence(
                        shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB).asProxy(),
                        climber.moveToPositionCommand(() -> ClimberPosition.CLIMB_PREP).asProxy()));
    }

    public static Command moveToHomeCommand(Intake intake, Shooter shooter, ShooterTilt shooterTilt, Climber climber,
            NoteLift noteLift) {
        return Commands.sequence(
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB),
                climber.moveToPositionCommand(() -> ClimberPosition.BOTTOM),
                noteLift.moveToPositionCommand(() -> NoteLiftPosition.TOP),
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.BOTTOM),
                intake.moveToPositionCommand(() -> IntakePosition.TOP));
    }
}
