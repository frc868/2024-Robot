package frc.robot;

import static frc.robot.Constants.Shooter.PASSING_RPS;
import static frc.robot.Constants.Shooter.PODIUM_RPS;
import static frc.robot.Constants.Shooter.SUBWOOFER_RPS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

/**
 * Contains meta-commands that complete high-level tasks usign multiple
 * subsystems.
 */
public class RobotCommands {
    /**
     * Creates a command that targets the speaker with the drivetrain, spins the
     * shooter to the correct speed, and continuously tilts the shooter to the
     * correct angle.
     * 
     * @param drivetrain  the drivetrain
     * @param shooter     the shooter
     * @param shooterTilt the shooter tilt
     * @return the command
     */
    public static Command targetSpeakerCommand(Drivetrain drivetrain, Shooter shooter, ShooterTilt shooterTilt) {
        return Commands.parallel(
                drivetrain.targetSpeakerCommand(),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy()).withName("RobotCommands.targetSpeaker");
    }

    /**
     * Creates a command that only targets the speaker using the shooter and shooter
     * tilt. Used for auto, when trying to target the speaker during an active path.
     * 
     * @param drivetrain  the drivetrain
     * @param shooter     the shooter
     * @param shooterTilt the shooter tilt
     * @return the command
     */
    public static Command targetSpeakerAutoCommand(Drivetrain drivetrain, Shooter shooter, ShooterTilt shooterTilt) {
        return Commands.parallel(
                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy())
                .withName("RobotCommands.targetSpeakerAuto");
    }

    /**
     * Creates a command that runs the intake sequence while moving the shooter tilt
     * to the correct position.
     * 
     * @param intake      the intake
     * @param shooterTilt the shooter tilt
     * @return the command
     */
    public static Command intakeNoteCommand(Intake intake, ShooterTilt shooterTilt) {
        return intake.intakeNoteCommand().asProxy()
                .alongWith(shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.INTAKE).asProxy())
                .withName("RobotCommands.intakeNote");
    }

    /**
     * Creates a command that runs the intake sequence for auto (lower speed) while
     * moving the shooter tilt to the correct position.
     * 
     * @param intake      the intake
     * @param shooterTilt the shooter tilt
     * @return
     */
    public static Command intakeNoteAutoCommand(Intake intake, ShooterTilt shooterTilt) {
        return intake.intakeNoteAutoCommand().asProxy()
                .alongWith(shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.INTAKE).asProxy())
                .withName("RobotCommands.intakeNoteAuto");
    }

    /**
     * Creates a command that moves a note into the amp position of the intake
     * depending on if we already have a note indexed.
     * 
     * @param intake      the intake
     * @param shooterTilt the shooter tilt
     * @return the command
     */
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

    /**
     * Creates a command that targets the speaker with the drivetrain, spins the
     * shooter to the correct speed, and continuously tilts the shooter to the
     * correct angle, while the robot is translating in any direction.
     * 
     * @param drivetrain  the drivetrain
     * @param shooter     the shooter
     * @param shooterTilt the shooter tilt
     * @return the command
     */
    public static Command targetSpeakerOnTheMoveCommand(Drivetrain drivetrain, Shooter shooter,
            ShooterTilt shooterTilt) {
        return Commands.parallel(
                drivetrain.targetPoseCommand(drivetrain::calculateEffectiveTargetLocation),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose,
                        drivetrain::calculateEffectiveTargetLocation).asProxy(),
                shooter.targetPoseCommand(drivetrain::getPose, drivetrain::calculateEffectiveTargetLocation)
                        .asProxy())
                .withName("RobotCommands.targetSpeakerOnTheMove");
    }

    /**
     * Creates a command that targets the speaker using manual setpoints for a shot
     * from the subwoofer.
     * 
     * @param drivetrain  the drivetrain
     * @param shooter     the shooter
     * @param shooterTilt the shooter tilt
     * @return the command
     */
    public static Command targetSpeakerSubwooferCommand(Drivetrain drivetrain, Shooter shooter,
            ShooterTilt shooterTilt) {
        return Commands.parallel(
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.SUBWOOFER).asProxy(),
                shooter.spinAtVelocityCommand(() -> SUBWOOFER_RPS).asProxy())
                .withName("RobotCommands.targetSpeakerSubwoofer");
    }

    /**
     * Creates a command that targets the speaker using manual setpoints for a shot
     * from the podium.
     * 
     * @param drivetrain  the drivetrain
     * @param shooter     the shooter
     * @param shooterTilt the shooter tilt
     * @return the command
     */
    public static Command targetSpeakerPodiumCommand(Drivetrain drivetrain, Shooter shooter,
            ShooterTilt shooterTilt) {
        return Commands.parallel(
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.PODIUM).asProxy(),
                shooter.spinAtVelocityCommand(() -> PODIUM_RPS).asProxy())
                .withName("RobotCommands.targetSpeakerPodium");
    }

    /**
     * Creates a command that sets the drivetrain, shooter, and shooter tilt to
     * target for a passing shot from the other side of the field, based on the
     * alliance color.
     * 
     * @param drivetrain  the drivetrain
     * @param shooter     the shooter
     * @param shooterTilt the shooter tilt
     * @return the command
     */
    public static Command targetPassCommand(Drivetrain drivetrain, Shooter shooter,
            ShooterTilt shooterTilt) {
        return Commands.parallel(
                drivetrain.controlledRotateCommand(
                        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 2.5 : -2.5),
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.PASS).asProxy(),
                shooter.spinAtVelocityCommand(() -> PASSING_RPS).asProxy())
                .withName("RobotCommands.targetPass");
    }

    /**
     * Creates a command that runs the intake rollers when the shooter is ready,
     * while continuing to target the speaker.
     * 
     * @param drivetrain  the drivetrain
     * @param intake      the intake
     * @param shooter     the shooter
     * @param shooterTilt the shooter tilt
     * @return the command
     */
    public static Command shootCommand(Drivetrain drivetrain, Intake intake, Shooter shooter, ShooterTilt shooterTilt) {
        return Commands.deadline(
                Commands.waitUntil(() -> shooter.atGoal() && shooterTilt.atGoal())
                        .andThen(intake.runRollersCommand().withTimeout(0.5)),
                drivetrain.targetSpeakerCommand(),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy())
                .withName("RobotCommands.shoot");
    }

    /**
     * Creates a command that runs the intake rollers when the shooter is ready,
     * while manually targetting the speaker from the subwoofer.
     * 
     * @param drivetrain  the drivetrain
     * @param intake      the intake
     * @param shooter     the shooter
     * @param shooterTilt the shooter tilt
     * @return the command
     */
    public static Command shootSubwooferCommand(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt) {
        return Commands.deadline(
                Commands.waitUntil(() -> shooter.atGoal() && shooterTilt.atGoal())
                        .andThen(intake.runRollersCommand().withTimeout(0.5)),
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.SUBWOOFER).asProxy(),
                shooter.spinAtVelocityCommand(() -> SUBWOOFER_RPS).asProxy())
                .withName("RobotCommands.shootSubwoofer");
    }

    /**
     * Creates a command that runs the intake rollers when the shooter is ready,
     * while manually targetting the speaker from the podium.
     * 
     * @param drivetrain  the drivetrain
     * @param intake      the intake
     * @param shooter     the shooter
     * @param shooterTilt the shooter tilt
     * @return the command
     */
    public static Command shootPodiumCommand(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt) {
        return Commands.deadline(
                Commands.waitUntil(() -> shooter.atGoal() && shooterTilt.atGoal())
                        .andThen(intake.runRollersCommand().withTimeout(0.5)),
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.PODIUM).asProxy(),
                shooter.spinAtVelocityCommand(() -> PODIUM_RPS).asProxy())
                .withName("RobotCommands.shootPodium");
    }

    /**
     * Creates a command that runs the intake rollers when the shooter is ready,
     * while continuing to target the speaker, but without the drivetrain. Used for
     * auto, when trying to shoot a note during an active path.
     * 
     * @param drivetrain  the drivetrain
     * @param intake      the intake
     * @param shooter     the shooter
     * @param shooterTilt the shooter tilt
     * @return the command
     */
    public static Command shootAutoCommand(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt) {
        return Commands.deadline(
                Commands.waitUntil(() -> shooter.atGoal() && shooterTilt.atGoal())
                        .andThen(intake.runRollersCommand().withTimeout(0.5)),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy())
                .withName("RobotCommands.shoot");
    }

    /**
     * Creates a command that runs the intake rollers when the shooter is ready,
     * while continuing to target the speaker, and while the robot is translating in
     * any direction.
     * 
     * @param drivetrain  the drivetrain
     * @param intake      the intake
     * @param shooter     the shooter
     * @param shooterTilt the shooter tilt
     * @return the command
     */
    public static Command shootOnTheMoveCommand(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt) {
        return Commands.parallel(
                drivetrain.targetPoseCommand(drivetrain::calculateEffectiveTargetLocation),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose,
                        drivetrain::calculateEffectiveTargetLocation).asProxy(),
                shooter.targetPoseCommand(drivetrain::getPose,
                        drivetrain::calculateEffectiveTargetLocation).asProxy(),
                Commands.waitUntil(() -> shooter.atGoal() && shooterTilt.atGoal())
                        .andThen(intake.runRollersCommand().withTimeout(1)))
                .withName("RobotCommands.shootOnTheMove");
    }

    /**
     * Creates a command that moves mechanisms to make room for a note to be
     * inserted into the note lift.
     * 
     * @param shooter     the shooter
     * @param shooterTilt the shooter tilt
     * @param noteLift    the note lift
     * @param leds        the LEDs
     * @return the command
     */
    public static Command intakeToNoteLift(Shooter shooter, ShooterTilt shooterTilt, NoteLift noteLift, LEDs leds) {
        // proxying to allow shooterTilt to hold position while note lift moves down
        return Commands.sequence(
                new ScheduleCommand(leds.requestStateCommand(LEDState.FLASHING_WHITE).withTimeout(5)),
                new ScheduleCommand(shooter.stopCommand()),
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB).asProxy(),
                noteLift.moveToPositionCommand(() -> NoteLiftPosition.INTAKE).asProxy());
    }

    /**
     * Creates a command that shuts off the shooter and moves all mechanisms into
     * the correct positions for climbing with a note in the note lift.
     * 
     * @param intake      the intake
     * @param shooter     the shooter
     * @param shooterTilt the shooter tilt
     * @param climber     the climber
     * @param noteLift    the note lift
     * @return the command
     */
    public static Command prepareClimb(Intake intake, Shooter shooter, ShooterTilt shooterTilt, Climber climber,
            NoteLift noteLift) {
        return Commands.parallel(
                new ScheduleCommand(shooter.stopCommand()),
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB).asProxy(),
                intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy(),
                noteLift.moveToPositionCommand(() -> NoteLiftPosition.CLIMB_PREP).asProxy(),
                climber.moveToPositionCommand(() -> ClimberPosition.CLIMB_PREP).asProxy());
    }

    /**
     * Creates a command that shuts off the shooter and moves all mechanisms into
     * the correct positions for climbing without a note in the note lift, allowing
     * for a much larger range to climb on.
     * 
     * @param intake      the intake
     * @param shooter     the shooter
     * @param shooterTilt the shooter tilt
     * @param climber     the climber
     * @param noteLift    the note lift
     * @return the command
     */
    public static Command prepareQuickClimb(Intake intake, Shooter shooter, ShooterTilt shooterTilt, Climber climber,
            NoteLift noteLift) {
        return Commands.parallel(
                new ScheduleCommand(shooter.stopCommand()),
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB).asProxy(),
                intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy(),
                noteLift.moveToPositionCommand(() -> NoteLiftPosition.TOP).asProxy(),
                climber.moveToPositionCommand(() -> ClimberPosition.MAX_HEIGHT).asProxy());
    }

    /**
     * Creates a command that allows for re-climbing if necessary, without resetting
     * the rest of the robot.
     * 
     * @param intake      the intake
     * @param shooter     the shooter
     * @param shooterTilt the shooter tilt
     * @param climber     the climber
     * @return the command
     */
    public static Command resetClimb(Intake intake, Shooter shooter, ShooterTilt shooterTilt, Climber climber) {
        return Commands.sequence(
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB).asProxy(),
                climber.moveToPositionCommand(() -> ClimberPosition.BOTTOM).asProxy());
    }

    /**
     * Creates a command that de-climbs the robot, moving the climber back up so
     * that the robot moves to the ground.
     * 
     * @param intake      the intake
     * @param shooter     the shooter
     * @param shooterTilt the shooter tilt
     * @param climber     the climber
     * @return the command
     */
    public static Command deClimb(Intake intake, Shooter shooter, ShooterTilt shooterTilt, Climber climber) {
        return Commands.sequence(
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB).asProxy(),
                climber.moveToPositionCommand(() -> ClimberPosition.CLIMB_PREP).asProxy());
    }

    /**
     * Creates a command that moves all mechanisms to their homing positions (useful
     * before powering off).
     * 
     * @param intake      the intake
     * @param shooter     the shooter
     * @param shooterTilt the shooter tilt
     * @param climber     the climber
     * @return the command
     */
    public static Command moveToHomeCommand(Intake intake, Shooter shooter, ShooterTilt shooterTilt, Climber climber) {
        return Commands.sequence(
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.CLIMB).asProxy(),
                climber.moveToPositionCommand(() -> ClimberPosition.BOTTOM).asProxy(),
                shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.BOTTOM).asProxy(),
                intake.moveToPositionCommand(() -> IntakePosition.TOP).asProxy());
    }
}
