package frc.robot;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.techhounds.houndutil.houndauto.AutoRoutine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Intake.IntakePosition;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTilt;

public class Autos {
    public static AutoRoutine autoTest(Drivetrain drivetrain, Intake intake,
            Shooter shooter, ShooterTilt shooterTilt) {
        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("test.1");

        Command command = Commands.parallel(
                Commands.sequence(
                        drivetrain.followPathCommand(path)));

        return new AutoRoutine("test", command,
                List.of(path),
                new Pose2d(path.getPreviewStartingHolonomicPose().getX(), path.getPreviewStartingHolonomicPose().getY(),
                        new Rotation2d(Math.PI)));
    }

    public static AutoRoutine autoCBA(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt) {
        PathPlannerPath pathStartToC = PathPlannerPath.fromChoreoTrajectory("CBA.1");
        PathPlannerPath pathCToB = PathPlannerPath.fromChoreoTrajectory("CBA.2");
        PathPlannerPath pathBToA = PathPlannerPath.fromChoreoTrajectory("CBA.3");
        Pose2d startingPose = pathStartToC.getPreviewStartingHolonomicPose();

        Command command = Commands.sequence(
                Commands.runOnce(drivetrain::stop, drivetrain),
                RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt)
                        .deadlineWith(intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy()),
                drivetrain.followPathCommand(pathStartToC).andThen(Commands.waitSeconds(0))
                        .deadlineWith(
                                intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy(),
                                intake.runRollersCommand(),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy()),
                drivetrain.followPathCommand(pathCToB).andThen(Commands.waitSeconds(0))
                        .deadlineWith(
                                intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy(),
                                intake.runRollersCommand(),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy()),
                drivetrain.followPathCommand(pathBToA).andThen(Commands.waitSeconds(0.5))
                        .deadlineWith(
                                intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy(),
                                intake.runRollersCommand(),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy()));

        return new AutoRoutine("CBA", command,
                List.of(pathStartToC, pathCToB, pathBToA),
                new Pose2d(startingPose.getX(), startingPose.getY(), new Rotation2d(Math.PI)));
    }

    public static AutoRoutine autoCBA1(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt) {
        PathPlannerPath pathStartToC = PathPlannerPath.fromChoreoTrajectory("CBA12.1");
        PathPlannerPath pathCToB = PathPlannerPath.fromChoreoTrajectory("CBA12.2");
        PathPlannerPath pathBToA = PathPlannerPath.fromChoreoTrajectory("CBA12.3");
        PathPlannerPath pathATo1 = PathPlannerPath.fromChoreoTrajectory("CBA12.4");
        PathPlannerPath path1ToScore = PathPlannerPath.fromChoreoTrajectory("CBA12.5");
        PathPlannerPath pathScoreTo2 = PathPlannerPath.fromChoreoTrajectory("CBA12.6");
        PathPlannerPath path2ToScore = PathPlannerPath.fromChoreoTrajectory("CBA12.7");
        Pose2d startingPose = pathStartToC.getPreviewStartingHolonomicPose();

        Command command = Commands.sequence(
                Commands.runOnce(drivetrain::stop, drivetrain),
                RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt)
                        .deadlineWith(intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy()),
                drivetrain.followPathCommand(pathStartToC)
                        .andThen(Commands.waitSeconds(0.75))
                        .deadlineWith(
                                intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy(),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose,
                                        drivetrain::calculateEffectiveTargetLocation).asProxy(),
                                shooter.targetSpeakerCommand(drivetrain::getPose,
                                        drivetrain::calculateEffectiveTargetLocation).asProxy(),
                                Commands.waitUntil(shooter::atGoal).andThen(intake.runRollersCommand())),
                drivetrain.followPathCommand(pathCToB).andThen(Commands.waitSeconds(0))
                        .deadlineWith(
                                intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy(),
                                intake.runRollersCommand(),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy()),
                drivetrain.followPathCommand(pathBToA).andThen(Commands.waitSeconds(0.5))
                        .deadlineWith(
                                intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy(),
                                intake.runRollersCommand(),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy()),

                drivetrain.followPathCommand(pathATo1)
                        .deadlineWith(RobotCommands.intakeNoteAutoCommand(intake, shooterTilt)),
                drivetrain.followPathCommand(path1ToScore)
                        .deadlineWith(
                                Commands.sequence(
                                        intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy()
                                                .alongWith(intake.runRollersCommand()
                                                        .until(intake.noteInShooterTrigger))),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy()),
                Commands.waitSeconds(0.75).andThen(intake.runRollersCommand().withTimeout(0.25)).deadlineWith(
                        shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                        shooter.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                        drivetrain.standaloneTargetSpeakerCommand()),

                drivetrain.followPathCommand(pathScoreTo2)
                        .deadlineWith(RobotCommands.intakeNoteAutoCommand(intake, shooterTilt)),
                drivetrain.followPathCommand(path2ToScore)
                        .deadlineWith(
                                Commands.sequence(
                                        intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy()
                                                .alongWith(intake.runRollersCommand()
                                                        .until(intake.noteInShooterTrigger))),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy()),
                Commands.waitSeconds(0.75).andThen(intake.runRollersCommand().withTimeout(0.25)).deadlineWith(
                        shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                        shooter.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                        drivetrain.standaloneTargetSpeakerCommand()));

        return new AutoRoutine("CBA1", command,
                List.of(pathStartToC, pathCToB, pathBToA, pathATo1, path1ToScore, pathScoreTo2, path2ToScore),
                new Pose2d(startingPose.getX(), startingPose.getY(), new Rotation2d(Math.PI)));
    }

    public static AutoRoutine autoCBA12(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt) {
        PathPlannerPath pathStartToC = PathPlannerPath.fromChoreoTrajectory("CBA12.1");
        PathPlannerPath pathCToB = PathPlannerPath.fromChoreoTrajectory("CBA12.2");
        PathPlannerPath pathBToA = PathPlannerPath.fromChoreoTrajectory("CBA12.3");
        PathPlannerPath pathATo1 = PathPlannerPath.fromChoreoTrajectory("CBA12.4");
        PathPlannerPath path1ToScore = PathPlannerPath.fromChoreoTrajectory("CBA12.5");
        PathPlannerPath pathScoreTo2 = PathPlannerPath.fromChoreoTrajectory("CBA12.6");
        PathPlannerPath path2ToScore = PathPlannerPath.fromChoreoTrajectory("CBA12.7");
        Pose2d startingPose = pathStartToC.getPreviewStartingHolonomicPose();

        Command command = Commands.sequence(
                Commands.runOnce(drivetrain::stop, drivetrain),
                RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt)
                        .deadlineWith(intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy()),
                drivetrain.followPathCommand(pathStartToC)
                        .andThen(Commands.waitSeconds(0.1))
                        .deadlineWith(
                                intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy(),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose,
                                        drivetrain::calculateEffectiveTargetLocation).asProxy(),
                                shooter.targetSpeakerCommand(drivetrain::getPose,
                                        drivetrain::calculateEffectiveTargetLocation).asProxy(),
                                Commands.waitUntil(shooter::atGoal).andThen(intake.runRollersCommand())),
                drivetrain.followPathCommand(pathCToB).andThen(Commands.waitSeconds(0))
                        .deadlineWith(
                                intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy(),
                                intake.runRollersCommand(),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy()),
                drivetrain.followPathCommand(pathBToA).andThen(Commands.waitSeconds(0.5))
                        .deadlineWith(
                                intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy(),
                                intake.runRollersCommand(),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy()),

                drivetrain.followPathCommand(pathATo1)
                        .deadlineWith(RobotCommands.intakeNoteAutoCommand(intake, shooterTilt)),
                drivetrain.followPathCommand(path1ToScore)
                        .deadlineWith(
                                RobotCommands.intakeNoteAutoCommand(intake, shooterTilt),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy()),
                Commands.waitSeconds(0.3).andThen(intake.runRollersCommand().withTimeout(0.25)).deadlineWith(
                        shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                        shooter.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                        drivetrain.standaloneTargetSpeakerCommand()),

                drivetrain.followPathCommand(pathScoreTo2)
                        .deadlineWith(RobotCommands.intakeNoteAutoCommand(intake, shooterTilt)),
                drivetrain.followPathCommand(path2ToScore)
                        .deadlineWith(
                                RobotCommands.intakeNoteAutoCommand(intake, shooterTilt),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy()),
                Commands.waitSeconds(0.3).andThen(intake.runRollersCommand().withTimeout(0.25)).deadlineWith(
                        shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                        shooter.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                        drivetrain.standaloneTargetSpeakerCommand()));

        return new AutoRoutine("CBA12", command,
                List.of(pathStartToC, pathCToB, pathBToA, pathATo1, path1ToScore, pathScoreTo2, path2ToScore),
                new Pose2d(startingPose.getX(), startingPose.getY(), new Rotation2d(Math.PI)));
    }

    public static AutoRoutine auto213(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt) {
        PathPlannerPath pathStartTo2 = PathPlannerPath.fromChoreoTrajectory("213.1");
        PathPlannerPath path2ToScore = PathPlannerPath.fromChoreoTrajectory("213.2");
        PathPlannerPath pathScoreTo3 = PathPlannerPath.fromChoreoTrajectory("213.3");
        PathPlannerPath path3ToScore = PathPlannerPath.fromChoreoTrajectory("213.4");
        PathPlannerPath pathScoreTo1 = PathPlannerPath.fromChoreoTrajectory("213.5");
        PathPlannerPath path1ToScore = PathPlannerPath.fromChoreoTrajectory("213.6");
        Pose2d startingPose = pathStartTo2.getPreviewStartingHolonomicPose();

        Command command = Commands.sequence(
                RobotCommands.shootAutoCommand(drivetrain, intake, shooter, shooterTilt),

                drivetrain.followPathCommand(pathStartTo2)
                        .deadlineWith(
                                RobotCommands.intakeNoteAutoCommand(intake, shooterTilt)),
                drivetrain.followPathCommand(path2ToScore)
                        .deadlineWith(
                                RobotCommands.intakeNoteAutoCommand(intake, shooterTilt),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy()),
                Commands.waitSeconds(0.5).andThen(intake.runRollersCommand().withTimeout(0.5)).deadlineWith(
                        shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                        shooter.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                        drivetrain.standaloneTargetSpeakerCommand()),

                drivetrain.followPathCommand(pathScoreTo3)
                        .deadlineWith(
                                RobotCommands.intakeNoteAutoCommand(intake, shooterTilt)),
                drivetrain.followPathCommand(path3ToScore)
                        .deadlineWith(
                                RobotCommands.intakeNoteAutoCommand(intake, shooterTilt),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy()),
                Commands.waitSeconds(0.5).andThen(intake.runRollersCommand().withTimeout(0.5)).deadlineWith(
                        shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                        shooter.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                        drivetrain.standaloneTargetSpeakerCommand()),

                drivetrain.followPathCommand(pathScoreTo1)
                        .deadlineWith(
                                RobotCommands.intakeNoteAutoCommand(intake, shooterTilt)),
                drivetrain.followPathCommand(path1ToScore)
                        .deadlineWith(
                                RobotCommands.intakeNoteAutoCommand(intake, shooterTilt),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy()),
                Commands.waitSeconds(0.5).andThen(intake.runRollersCommand().withTimeout(0.5)).deadlineWith(
                        shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                        shooter.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                        drivetrain.standaloneTargetSpeakerCommand()));

        return new AutoRoutine("213", command,
                List.of(pathStartTo2, path2ToScore, pathScoreTo3, path3ToScore, pathScoreTo1, path1ToScore),
                new Pose2d(startingPose.getX(), startingPose.getY(), new Rotation2d(-2.297815857388982)));
    }

    public static AutoRoutine auto453(Drivetrain drivetrain, Intake intake,
            Shooter shooter, ShooterTilt shooterTilt) {
        PathPlannerPath pathStartTo4 = PathPlannerPath.fromChoreoTrajectory("453.1");
        PathPlannerPath path4ToScore = PathPlannerPath.fromChoreoTrajectory("453.2");
        PathPlannerPath pathScoreTo5 = PathPlannerPath.fromChoreoTrajectory("453.3");
        PathPlannerPath path5ToScore = PathPlannerPath.fromChoreoTrajectory("453.4");
        PathPlannerPath pathScoreTo3 = PathPlannerPath.fromChoreoTrajectory("453.5");
        PathPlannerPath path3ToScore = PathPlannerPath.fromChoreoTrajectory("453.6");

        Pose2d startingPose = pathStartTo4.getPreviewStartingHolonomicPose();

        Command command = Commands.sequence(
                RobotCommands.shootAutoCommand(drivetrain, intake, shooter, shooterTilt),

                drivetrain.followPathCommand(pathStartTo4)
                        .deadlineWith(
                                RobotCommands.intakeNoteAutoCommand(intake, shooterTilt)),
                drivetrain.followPathCommand(path4ToScore)
                        .deadlineWith(
                                RobotCommands.intakeNoteAutoCommand(intake, shooterTilt),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy()),
                Commands.waitSeconds(0.5).andThen(intake.runRollersCommand().withTimeout(0.5)).deadlineWith(
                        shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                        shooter.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                        drivetrain.standaloneTargetSpeakerCommand()),

                drivetrain.followPathCommand(pathScoreTo5)
                        .deadlineWith(
                                RobotCommands.intakeNoteAutoCommand(intake, shooterTilt)),
                drivetrain.followPathCommand(path5ToScore)
                        .deadlineWith(
                                RobotCommands.intakeNoteAutoCommand(intake, shooterTilt),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy()),
                Commands.waitSeconds(0.5).andThen(intake.runRollersCommand().withTimeout(0.5)).deadlineWith(
                        shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                        shooter.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                        drivetrain.standaloneTargetSpeakerCommand()));

        // drivetrain.followPathCommand(pathScoreTo3)
        // .deadlineWith(
        // intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy(),
        // intake.runRollersCommand().until(intake.getNoteInShooterTrigger())),
        // drivetrain.followPathCommand(path3ToScore)
        // .deadlineWith(
        // Commands.sequence(
        // intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy()
        // .alongWith(intake.runRollersCommand()
        // .until(intake.getNoteInShooterTrigger()))),
        // shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
        // shooter.targetSpeakerCommand(drivetrain::getPose).asProxy()),
        // Commands.waitSeconds(0.5).andThen(intake.runRollersCommand().withTimeout(0.5)).deadlineWith(
        // shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
        // shooter.targetSpeakerCommand(drivetrain::getPose).asProxy(),
        // drivetrain.standaloneTargetSpeakerCommand()));

        return new AutoRoutine("453", command,
                List.of(pathStartTo4, path4ToScore, pathScoreTo5, path5ToScore, pathScoreTo3,
                        path3ToScore),
                new Pose2d(startingPose.getX(), startingPose.getY(), new Rotation2d(2.146011906845138)));
    }
}
