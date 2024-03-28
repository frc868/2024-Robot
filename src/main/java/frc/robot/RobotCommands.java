package frc.robot;

import static frc.robot.Constants.Shooter.PODIUM_RPS;
import static frc.robot.Constants.Shooter.SUBWOOFER_RPS;

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
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.NoteLift;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTilt;
import frc.robot.subsystems.LEDs.LEDState;

public class RobotCommands {
    public static Command targetSpeakerCommand(Drivetrain drivetrain, Shooter shooter, ShooterTilt shooterTilt) {
        return Commands.parallel(
                drivetrain.targetSpeakerCommand(),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy()).withName("RobotCommands.targetSpeaker");
    }

    public static Command targetSpeakerAutoCommand(Drivetrain drivetrain, Shooter shooter, ShooterTilt shooterTilt) {
        return Commands.parallel(
                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy())
                .withName("RobotCommands.targetSpeakerAuto");
    }

    public static Command intakeNoteCommand(Intake intake, ShooterTilt shooterTilt) {
        return intake.intakeNoteCommand().asProxy()
                .alongWith(shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.INTAKE).asProxy())
                .withName("RobotCommands.intakeNote");
    }

    public static Command intakeNoteAutoCommand(Intake intake, ShooterTilt shooterTilt) {
        return intake.intakeNoteAutoCommand().asProxy()
                .alongWith(shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.INTAKE).asProxy())
                .withName("RobotCommands.intakeNoteAuto");
    }

    public static Command ampPrepIntakeCommand(Intake intake, ShooterTilt shooterTilt) {
        return Commands.either(
                Commands.sequence(
                        intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy(),
                        intake.reverseRollersCommand().until(intake.noteInIntakeFromShooterTrigger).asProxy(),
                        intake.moveToPositionCommand(() -> IntakePosition.AMP).asProxy()),
                Commands.sequence(
                        intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy(),
                        intake.runRollersCommand().until(intake.noteInIntakeFromOutsideTrigger).asProxy(),
                        intake.runRollersHalfCommand().until(intake.noteInShooterTrigger).asProxy(),
                        intake.runRollersSlowCommand().until(intake.noteFullyInShooterTrigger).asProxy(),
                        intake.reverseRollersCommand().until(intake.noteInIntakeFromShooterTrigger).asProxy(),
                        intake.moveToPositionCommand(() -> IntakePosition.AMP).asProxy()),
                intake.noteInShooterTrigger).withName("RobotCommands.ampPrepCommand");
    }

    public static Command targetSpeakerOnTheMoveCommand(Drivetrain drivetrain, Shooter shooter,
            ShooterTilt shooterTilt) {
        return Commands.parallel(
                drivetrain.targetSpeakerCommand(drivetrain::calculateEffectiveTargetLocation),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose,
                        drivetrain::calculateEffectiveTargetLocation).asProxy(),
                shooter.targetSpeakerCommand(drivetrain::getPose, drivetrain::calculateEffectiveTargetLocation)
                        .asProxy())
                .withName("RobotCommands.targetSpeakerOnTheMove");
    }

    public static Command targetSpeakerSubwooferCommand(Drivetrain drivetrain, Shooter shooter,
            ShooterTilt shooterTilt) {
        return Commands.parallel(
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.SUBWOOFER).asProxy(),
                shooter.spinAtVelocityCommand(() -> SUBWOOFER_RPS).asProxy())
                .withName("RobotCommands.targetSpeakerSubwoofer");
    }

    public static Command targetSpeakerPodiumCommand(Drivetrain drivetrain, Shooter shooter,
            ShooterTilt shooterTilt) {
        return Commands.parallel(
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.PODIUM).asProxy(),
                shooter.spinAtVelocityCommand(() -> PODIUM_RPS).asProxy())
                .withName("RobotCommands.targetSpeakerSubwoofer");
    }

    public static Command shootCommand(Drivetrain drivetrain, Intake intake, Shooter shooter, ShooterTilt shooterTilt) {
        return Commands.deadline(
                Commands.waitUntil(() -> shooter.atGoal() && shooterTilt.atGoal())
                        .andThen(intake.runRollersCommand().withTimeout(0.5)),
                drivetrain.targetSpeakerCommand(),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy())
                .withName("RobotCommands.shoot");
    }

    public static Command shootSubwooferCommand(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt) {
        return Commands.deadline(
                Commands.waitUntil(() -> shooter.atGoal() && shooterTilt.atGoal())
                        .andThen(intake.runRollersCommand().withTimeout(0.5)),
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.SUBWOOFER).asProxy(),
                shooter.spinAtVelocityCommand(() -> SUBWOOFER_RPS).asProxy())
                .withName("RobotCommands.shootSubwoofer");
    }

    public static Command shootPodiumCommand(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt) {
        return Commands.deadline(
                Commands.waitUntil(() -> shooter.atGoal() && shooterTilt.atGoal())
                        .andThen(intake.runRollersCommand().withTimeout(0.5)),
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.PODIUM).asProxy(),
                shooter.spinAtVelocityCommand(() -> PODIUM_RPS).asProxy())
                .withName("RobotCommands.shootPodium");
    }

    public static Command shootAutoCommand(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt) {
        return Commands.deadline(
                Commands.waitUntil(() -> shooter.atGoal() && shooterTilt.atGoal())
                        .andThen(intake.runRollersCommand().withTimeout(0.5)),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy())
                .withName("RobotCommands.shoot");
    }

    public static Command shootOnTheMoveCommand(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt) {
        return Commands.parallel(
                drivetrain.targetSpeakerCommand(drivetrain::calculateEffectiveTargetLocation),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose,
                        drivetrain::calculateEffectiveTargetLocation).asProxy(),
                shooter.targetSpeakerCommand(drivetrain::getPose,
                        drivetrain::calculateEffectiveTargetLocation).asProxy(),
                Commands.waitUntil(() -> shooter.atGoal() && shooterTilt.atGoal())
                        .andThen(intake.runRollersCommand().withTimeout(1)))
                .withName("RobotCommands.shootOnTheMove");
    }

    public static Command intakeToNoteLift(Shooter shooter, ShooterTilt shooterTilt, NoteLift noteLift, LEDs leds) {
        // proxying to allow shooterTilt to hold position while note lift moves down
        return Commands.sequence(
                new ScheduleCommand(leds.requestStateCommand(LEDState.FLASHING_WHITE).withTimeout(5)),
                new ScheduleCommand(shooter.stopCommand()),
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB).asProxy(),
                noteLift.moveToPositionCommand(() -> NoteLiftPosition.INTAKE).asProxy());
    }

    public static Command prepareClimb(Intake intake, Shooter shooter, ShooterTilt shooterTilt, Climber climber,
            NoteLift noteLift) {
        return Commands.parallel(
                new ScheduleCommand(shooter.stopCommand()),
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB).asProxy(),
                intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy(),
                noteLift.moveToPositionCommand(() -> NoteLiftPosition.CLIMB_PREP).asProxy(),
                climber.moveToPositionCommand(() -> ClimberPosition.CLIMB_PREP).asProxy());
    }

    public static Command prepareQuickClimb(Intake intake, Shooter shooter, ShooterTilt shooterTilt, Climber climber,
            NoteLift noteLift) {
        return Commands.parallel(
                new ScheduleCommand(shooter.stopCommand()),
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB).asProxy(),
                intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy(),
                noteLift.moveToPositionCommand(() -> NoteLiftPosition.TOP).asProxy(),
                climber.moveToPositionCommand(() -> ClimberPosition.MAX_HEIGHT).asProxy());
    }

    public static Command resetClimb(Intake intake, Shooter shooter, ShooterTilt shooterTilt, Climber climber,
            NoteLift noteLift) {
        return Commands.sequence(
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB).asProxy(),
                climber.moveToPositionCommand(() -> ClimberPosition.BOTTOM).asProxy());
    }

    public static Command deClimb(Intake intake, Shooter shooter, ShooterTilt shooterTilt, Climber climber,
            NoteLift noteLift) {
        return Commands.sequence(
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB).asProxy(),
                climber.moveToPositionCommand(() -> ClimberPosition.CLIMB_PREP).asProxy());
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

    public static Command homeMechanismsCommand(Intake intake, Shooter shooter, ShooterTilt shooterTilt,
            Climber climber, NoteLift noteLift) {
        return Commands.sequence(
                Commands.parallel(
                        shooterTilt.setInitializedCommand(false),
                        climber.setInitializedCommand(false)),
                shooterTilt.zeroMechanismCommand());
        // Commands.parallel(
        // climber.zeroMechanismCommand()));
    }
}
