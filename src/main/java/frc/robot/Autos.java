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
    public static Command driveIntakeShootCenterCommand(PathPlannerPath intakingPath, PathPlannerPath shootingPath,
            double delay, double shotTimeout, Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt) {
        return Commands.sequence(
                drivetrain.followPathCommand(intakingPath)
                        .deadlineWith(RobotCommands.intakeNoteAutoCommand(intake, shooterTilt)),
                drivetrain.followPathCommand(shootingPath)
                        .deadlineWith(
                                RobotCommands.intakeNoteAutoCommand(intake, shooterTilt),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                                shooter.targetSpeakerCommand(drivetrain::getPose).asProxy()),
                Commands.waitSeconds(delay).andThen(intake.runRollersCommand().withTimeout(shotTimeout)).deadlineWith(
                        shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                        shooter.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                        drivetrain.standaloneTargetSpeakerCommand()));
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

    public static AutoRoutine autoWaitCBA(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt) {
        PathPlannerPath pathStartToC = PathPlannerPath.fromChoreoTrajectory("CBA.1");
        PathPlannerPath pathCToB = PathPlannerPath.fromChoreoTrajectory("CBA.2");
        PathPlannerPath pathBToA = PathPlannerPath.fromChoreoTrajectory("CBA.3");
        Pose2d startingPose = pathStartToC.getPreviewStartingHolonomicPose();

        Command command = Commands.sequence(
                Commands.waitSeconds(7),
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

        return new AutoRoutine("WaitCBA", command,
                List.of(pathStartToC, pathCToB, pathBToA),
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
                                shooter.targetPoseCommand(drivetrain::getPose,
                                        drivetrain::calculateEffectiveTargetLocation).asProxy(),
                                Commands.waitUntil(shooter::atGoal).andThen(intake.runRollersCommand())),
                drivetrain.followPathCommand(pathCToB)
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

    public static AutoRoutine autoCBA123(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt) {
        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("CBA123.1");
        Pose2d startingPose = path.getPreviewStartingHolonomicPose();

        Command command = Commands.sequence(
                Commands.runOnce(drivetrain::stop, drivetrain),
                RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt)
                        .deadlineWith(intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy()),
                drivetrain.followPathCommand(path).alongWith(Commands.sequence(
                        Commands.waitSeconds(3.71).deadlineWith(
                                intake.moveToPositionCommand(() -> IntakePosition.GROUND).asProxy(),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose,
                                        drivetrain::calculateEffectiveTargetLocation).asProxy(),
                                shooter.targetPoseCommand(drivetrain::getPose,
                                        drivetrain::calculateEffectiveTargetLocation).asProxy(),
                                Commands.waitUntil(shooter::atGoal).andThen(intake.runRollersCommand())),

                        Commands.waitSeconds(5.22 - 3.71)
                                .deadlineWith(RobotCommands.intakeNoteAutoCommand(intake, shooterTilt)),

                        Commands.waitSeconds(7.07 - 5.22).deadlineWith(
                                RobotCommands.intakeNoteAutoCommand(intake, shooterTilt),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose,
                                        drivetrain::calculateEffectiveTargetLocation).asProxy(),
                                shooter.targetPoseCommand(drivetrain::getPose,
                                        drivetrain::calculateEffectiveTargetLocation).asProxy(),
                                Commands.waitSeconds(1.6).andThen(intake.runRollersCommand().withTimeout(0.25))),

                        Commands.waitSeconds(8.65 - 7.07)
                                .deadlineWith(RobotCommands.intakeNoteAutoCommand(intake, shooterTilt)),

                        Commands.waitSeconds(10.41 - 8.65).deadlineWith(
                                RobotCommands.intakeNoteAutoCommand(intake, shooterTilt),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose,
                                        drivetrain::calculateEffectiveTargetLocation).asProxy(),
                                shooter.targetPoseCommand(drivetrain::getPose,
                                        drivetrain::calculateEffectiveTargetLocation).asProxy(),
                                Commands.waitSeconds(1.51).andThen(intake.runRollersCommand().withTimeout(0.25))),

                        Commands.waitSeconds(12.35 - 10.41)
                                .deadlineWith(RobotCommands.intakeNoteAutoCommand(intake, shooterTilt)),

                        Commands.waitSeconds(14.15 - 12.35).deadlineWith(
                                RobotCommands.intakeNoteAutoCommand(intake, shooterTilt),
                                shooterTilt.targetSpeakerCommand(drivetrain::getPose,
                                        drivetrain::calculateEffectiveTargetLocation).asProxy(),
                                shooter.targetPoseCommand(drivetrain::getPose,
                                        drivetrain::calculateEffectiveTargetLocation).asProxy(),
                                Commands.waitSeconds(1.55).andThen(intake.runRollersCommand().withTimeout(0.25)))

                )));

        return new AutoRoutine("CBA123", command,
                List.of(path),
                new Pose2d(startingPose.getX(), startingPose.getY(), new Rotation2d(Math.PI)));
    }

    public static AutoRoutine auto123(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt) {
        PathPlannerPath pathStartTo1 = PathPlannerPath.fromChoreoTrajectory("123.1");
        PathPlannerPath path1ToScore = PathPlannerPath.fromChoreoTrajectory("123.2");
        PathPlannerPath pathScoreTo2 = PathPlannerPath.fromChoreoTrajectory("123.3");
        PathPlannerPath path2ToScore = PathPlannerPath.fromChoreoTrajectory("123.4");
        PathPlannerPath pathScoreTo3 = PathPlannerPath.fromChoreoTrajectory("123.5");
        PathPlannerPath path3ToScore = PathPlannerPath.fromChoreoTrajectory("123.6");
        Pose2d startingPose = pathStartTo1.getPreviewStartingHolonomicPose();

        Command command = Commands.sequence(
                RobotCommands.shootAutoCommand(drivetrain, intake, shooter, shooterTilt),
                driveIntakeShootCenterCommand(pathStartTo1, path1ToScore, 0.25, 0.25, drivetrain, intake, shooter,
                        shooterTilt),
                driveIntakeShootCenterCommand(pathScoreTo2, path2ToScore, 0.25, 0.25, drivetrain, intake, shooter,
                        shooterTilt),
                driveIntakeShootCenterCommand(pathScoreTo3, path3ToScore, 0.25, 0.25, drivetrain, intake, shooter,
                        shooterTilt));
        return new AutoRoutine("123", command,
                List.of(pathStartTo1, path1ToScore, pathScoreTo2, path2ToScore, pathScoreTo3, path3ToScore),
                new Pose2d(startingPose.getX(), startingPose.getY(), new Rotation2d(-2.1045045040359978)));
    }

    public static AutoRoutine auto231(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt) {
        PathPlannerPath pathStartTo2 = PathPlannerPath.fromChoreoTrajectory("231.1");
        PathPlannerPath path2ToScore = PathPlannerPath.fromChoreoTrajectory("231.2");
        PathPlannerPath pathScoreTo3 = PathPlannerPath.fromChoreoTrajectory("231.3");
        PathPlannerPath path3ToScore = PathPlannerPath.fromChoreoTrajectory("231.4");
        PathPlannerPath pathScoreTo1 = PathPlannerPath.fromChoreoTrajectory("231.5");
        PathPlannerPath path1ToScore = PathPlannerPath.fromChoreoTrajectory("231.6");
        Pose2d startingPose = pathStartTo2.getPreviewStartingHolonomicPose();

        Command command = Commands.sequence(
                RobotCommands.shootAutoCommand(drivetrain, intake, shooter, shooterTilt),
                driveIntakeShootCenterCommand(pathStartTo2, path2ToScore, 0.25, 0.25, drivetrain, intake, shooter,
                        shooterTilt),
                driveIntakeShootCenterCommand(pathScoreTo1, path1ToScore, 0.25, 0.25, drivetrain, intake, shooter,
                        shooterTilt),
                driveIntakeShootCenterCommand(pathScoreTo3, path3ToScore, 0.25, 0.25, drivetrain, intake, shooter,
                        shooterTilt));
        return new AutoRoutine("231", command,
                List.of(pathStartTo2, path2ToScore, pathScoreTo3, path3ToScore, pathScoreTo1, path1ToScore),
                new Pose2d(startingPose.getX(), startingPose.getY(), new Rotation2d(-2.1045045040359978)));
    }

    public static AutoRoutine auto345(Drivetrain drivetrain, Intake intake,
            Shooter shooter, ShooterTilt shooterTilt) {
        PathPlannerPath pathStartTo3 = PathPlannerPath.fromChoreoTrajectory("345.1");
        PathPlannerPath path3ToScore = PathPlannerPath.fromChoreoTrajectory("345.2");
        PathPlannerPath pathScoreTo4 = PathPlannerPath.fromChoreoTrajectory("345.3");
        PathPlannerPath path4ToScore = PathPlannerPath.fromChoreoTrajectory("345.4");
        PathPlannerPath pathScoreTo5 = PathPlannerPath.fromChoreoTrajectory("345.5");
        PathPlannerPath path5ToScore = PathPlannerPath.fromChoreoTrajectory("345.6");

        Pose2d startingPose = pathStartTo3.getPreviewStartingHolonomicPose();

        Command command = Commands.sequence(
                RobotCommands.shootAutoCommand(drivetrain, intake, shooter, shooterTilt),

                driveIntakeShootCenterCommand(pathStartTo3, path3ToScore, 0.25, 0.25, drivetrain, intake, shooter,
                        shooterTilt),
                driveIntakeShootCenterCommand(pathScoreTo4, path4ToScore, 0.25, 0.25, drivetrain, intake, shooter,
                        shooterTilt),
                driveIntakeShootCenterCommand(pathScoreTo5, path5ToScore, 0.25, 0.25, drivetrain, intake, shooter,
                        shooterTilt));

        return new AutoRoutine("345", command,
                List.of(pathStartTo3, path3ToScore, pathScoreTo4, path4ToScore, pathScoreTo5,
                        path5ToScore),
                new Pose2d(startingPose.getX(), startingPose.getY(), new Rotation2d(2.146011906845138)));
    }

    public static AutoRoutine auto543(Drivetrain drivetrain, Intake intake,
            Shooter shooter, ShooterTilt shooterTilt) {
        PathPlannerPath pathStartTo5 = PathPlannerPath.fromChoreoTrajectory("543.1");
        PathPlannerPath path5ToScore = PathPlannerPath.fromChoreoTrajectory("543.2");
        PathPlannerPath pathScoreTo4 = PathPlannerPath.fromChoreoTrajectory("543.3");
        PathPlannerPath path4ToScore = PathPlannerPath.fromChoreoTrajectory("543.4");
        PathPlannerPath pathScoreTo3 = PathPlannerPath.fromChoreoTrajectory("543.5");
        PathPlannerPath path3ToScore = PathPlannerPath.fromChoreoTrajectory("543.6");

        Pose2d startingPose = pathStartTo5.getPreviewStartingHolonomicPose();

        Command command = Commands.sequence(
                RobotCommands.shootAutoCommand(drivetrain, intake, shooter, shooterTilt),

                driveIntakeShootCenterCommand(pathStartTo5, path5ToScore, 0.25, 0.15, drivetrain, intake, shooter,
                        shooterTilt),
                driveIntakeShootCenterCommand(pathScoreTo4, path4ToScore, 0.25, 0.15, drivetrain, intake, shooter,
                        shooterTilt),
                driveIntakeShootCenterCommand(pathScoreTo3, path3ToScore, 0.25, 0.15, drivetrain, intake, shooter,
                        shooterTilt));

        return new AutoRoutine("543", command,
                List.of(pathStartTo5, path5ToScore, pathScoreTo4, path4ToScore, pathScoreTo3, path3ToScore),
                new Pose2d(startingPose.getX(), startingPose.getY(), new Rotation2d(2.146011906845138)));
    }

    public static AutoRoutine auto435(Drivetrain drivetrain, Intake intake,
            Shooter shooter, ShooterTilt shooterTilt) {
        PathPlannerPath pathStartTo4 = PathPlannerPath.fromChoreoTrajectory("435.1");
        PathPlannerPath path4ToScore = PathPlannerPath.fromChoreoTrajectory("435.2");
        PathPlannerPath pathScoreTo3 = PathPlannerPath.fromChoreoTrajectory("435.3");
        PathPlannerPath path3ToScore = PathPlannerPath.fromChoreoTrajectory("435.4");
        PathPlannerPath pathScoreTo5 = PathPlannerPath.fromChoreoTrajectory("435.5");
        PathPlannerPath path5ToScore = PathPlannerPath.fromChoreoTrajectory("435.6");

        Pose2d startingPose = pathStartTo4.getPreviewStartingHolonomicPose();

        Command command = Commands.sequence(
                RobotCommands.shootAutoCommand(drivetrain, intake, shooter, shooterTilt),

                driveIntakeShootCenterCommand(pathStartTo4, path4ToScore, 0.25, 0.15, drivetrain, intake, shooter,
                        shooterTilt),
                driveIntakeShootCenterCommand(pathScoreTo3, path3ToScore, 0.25, 0.15, drivetrain, intake, shooter,
                        shooterTilt),
                driveIntakeShootCenterCommand(pathScoreTo5, path5ToScore, 0.25, 0.15, drivetrain, intake, shooter,
                        shooterTilt));

        return new AutoRoutine("435", command,
                List.of(pathStartTo4, path4ToScore, pathScoreTo3, path3ToScore, pathScoreTo5,
                        path5ToScore),
                new Pose2d(startingPose.getX(), startingPose.getY(), new Rotation2d(2.146011906845138)));
    }
}
