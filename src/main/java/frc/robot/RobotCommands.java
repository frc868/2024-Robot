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

public class RobotCommands {
    public static Command targetSpeakerCommand(Drivetrain drivetrain, Shooter shooter, ShooterTilt shooterTilt) {
        return Commands.parallel(
                drivetrain.targetSpeakerCommand(),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose),
                shooter.targetSpeakerCommand(drivetrain::getPose)).withName("RobotCommands.targetSpeaker");
    }

    public static Command targetSpeakerAutoCommand(Drivetrain drivetrain, Shooter shooter, ShooterTilt shooterTilt) {
        return Commands.parallel(
                shooterTilt.targetSpeakerCommand(drivetrain::getPose),
                shooter.targetSpeakerCommand(drivetrain::getPose)).withName("RobotCommands.targetSpeakerAuto");
    }

    public static Command intakeNoteCommand(Intake intake, ShooterTilt shooterTilt) {
        return intake.intakeNoteCommand()
                .alongWith(shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.INTAKE))
                .withName("RobotCommands.intakeNote");
    }

    public static Command intakeNoteAutoCommand(Intake intake, ShooterTilt shooterTilt) {
        return intake.intakeNoteAutoCommand()
                .alongWith(shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.INTAKE))
                .withName("RobotCommands.intakeNoteAuto");
    }

    public static Command targetSpeakerOnTheMoveCommand(Drivetrain drivetrain, Shooter shooter,
            ShooterTilt shooterTilt) {
        return Commands.parallel(
                drivetrain.targetSpeakerCommand(drivetrain::calculateEffectiveTargetLocation),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose,
                        drivetrain::calculateEffectiveTargetLocation),
                shooter.targetSpeakerCommand(drivetrain::getPose, drivetrain::calculateEffectiveTargetLocation))
                .withName("RobotCommands.targetSpeakerOnTheMove");
    }

    public static Command targetFromSubwooferCommand(Drivetrain drivetrain, Shooter shooter, ShooterTilt shooterTilt) {
        return Commands.parallel(
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.SUBWOOFER),
                shooter.spinAtVelocityCommand(() -> BASE_SHOOTING_RPS)).withName("RobotCommands.targetFromSubwoofer");
    }

    public static Command shootCommand(Drivetrain drivetrain, Intake intake, Shooter shooter, ShooterTilt shooterTilt) {
        return Commands.deadline(
                Commands.waitUntil(() -> shooter.atGoal() && shooterTilt.atGoal())
                        .andThen(intake.runRollersCommand().withTimeout(1)),
                // drivetrain.targetSpeakerCommand(),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose),
                shooter.targetSpeakerCommand(drivetrain::getPose))
                .withName("RobotCommands.shoot");
    }

    public static Command shootOnTheMoveCommand(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt) {
        return Commands.parallel(
                drivetrain.targetSpeakerCommand(drivetrain::calculateEffectiveTargetLocation),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose,
                        drivetrain::calculateEffectiveTargetLocation),
                shooter.targetSpeakerCommand(drivetrain::getPose,
                        drivetrain::calculateEffectiveTargetLocation),
                Commands.waitUntil(() -> shooter.atGoal() && shooterTilt.atGoal())
                        .andThen(intake.runRollersCommand().withTimeout(1)))
                .withName("RobotCommands.shootOnTheMove");
    }

    public static Command intakeToNoteLift(Shooter shooter, ShooterTilt shooterTilt, NoteLift noteLift) {
        // proxying to allow shooterTilt to hold position while note lift moves down
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
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB),
                intake.moveToPositionCommand(() -> IntakePosition.GROUND),
                noteLift.moveToPositionCommand(() -> NoteLiftPosition.CLIMB_PREP),
                climber.moveToPositionCommand(() -> ClimberPosition.CLIMB_PREP));
    }

    public static Command resetClimb(Intake intake, Shooter shooter, ShooterTilt shooterTilt, Climber climber,
            NoteLift noteLift) {
        return Commands.sequence(
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB),
                climber.moveToPositionCommand(() -> ClimberPosition.BOTTOM));
    }

    public static Command deClimb(Intake intake, Shooter shooter, ShooterTilt shooterTilt, Climber climber,
            NoteLift noteLift) {
        return Commands.sequence(
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB),
                climber.moveToPositionCommand(() -> ClimberPosition.CLIMB_PREP));
    }

    public static Command moveToHomeCommand(Intake intake, Shooter shooter, ShooterTilt shooterTilt, Climber climber,
            NoteLift noteLift) {
        return Commands.sequence(
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB).asProxy(),
                climber.moveToPositionCommand(() -> ClimberPosition.BOTTOM).asProxy(),
                noteLift.moveToPositionCommand(() -> NoteLiftPosition.TOP).asProxy(),
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.BOTTOM).asProxy(),
                intake.moveToPositionCommand(() -> IntakePosition.TOP).asProxy());
    }
}
