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

    public static AutoRoutine autoA123(Drivetrain drivetrain, Intake intake, Shooter shooter, ShooterTilt shooterTilt) {
        PathPlannerPath pathStartToA = PathPlannerPath.fromChoreoTrajectory("A123.1");
        PathPlannerPath pathATo1 = PathPlannerPath.fromChoreoTrajectory("A123.2");
        PathPlannerPath path1ToScore = PathPlannerPath.fromChoreoTrajectory("A123.3");
        PathPlannerPath pathScoreTo2 = PathPlannerPath.fromChoreoTrajectory("A123.4");
        PathPlannerPath path2ToScore = PathPlannerPath.fromChoreoTrajectory("A123.5");
        PathPlannerPath pathScoreTo3 = PathPlannerPath.fromChoreoTrajectory("A123.6");
        PathPlannerPath path3ToScore = PathPlannerPath.fromChoreoTrajectory("A123.7");

        Command command = Commands.parallel(
                Commands.sequence(
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt)
                                .alongWith(intake.moveToPositionCommand(() -> IntakePosition.GROUND)),
                        drivetrain.followPathCommand(pathStartToA).alongWith(intake.intakeNoteAutoCommand()),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt),
                        drivetrain.followPathCommand(pathATo1).alongWith(intake.intakeNoteAutoCommand()),
                        drivetrain.followPathCommand(path1ToScore),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt),
                        drivetrain.followPathCommand(pathScoreTo2).alongWith(intake.intakeNoteAutoCommand()),
                        drivetrain.followPathCommand(path2ToScore),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt),
                        drivetrain.followPathCommand(pathScoreTo3).alongWith(intake.intakeNoteAutoCommand()),
                        drivetrain.followPathCommand(path3ToScore),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt)),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose));

        return new AutoRoutine("A123", command,
                List.of(pathStartToA, pathATo1, path1ToScore, pathScoreTo2, path2ToScore, pathScoreTo3, path3ToScore),
                new Pose2d(1.775, 6.012, new Rotation2d(-2.648)));
    }

    public static AutoRoutine autoBA123(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt) {
        PathPlannerPath pathStartToB = PathPlannerPath.fromChoreoTrajectory("BA123.1");
        PathPlannerPath pathBToA = PathPlannerPath.fromChoreoTrajectory("BA123.2");
        PathPlannerPath pathATo1 = PathPlannerPath.fromChoreoTrajectory("BA123.3");
        PathPlannerPath path1ToScore = PathPlannerPath.fromChoreoTrajectory("BA123.4");
        PathPlannerPath pathScoreTo2 = PathPlannerPath.fromChoreoTrajectory("BA123.5");
        PathPlannerPath path2ToScore = PathPlannerPath.fromChoreoTrajectory("BA123.6");
        PathPlannerPath pathScoreTo3 = PathPlannerPath.fromChoreoTrajectory("BA123.7");
        PathPlannerPath path3ToScore = PathPlannerPath.fromChoreoTrajectory("BA123.8");

        Pose2d startingPose = pathStartToB.getPreviewStartingHolonomicPose();

        Command command = Commands.parallel(
                Commands.sequence(
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt)
                                .alongWith(intake.moveToPositionCommand(() -> IntakePosition.GROUND)),
                        drivetrain.followPathCommand(pathStartToB).alongWith(intake.intakeNoteAutoCommand()),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt),
                        drivetrain.followPathCommand(pathBToA).alongWith(intake.intakeNoteAutoCommand()),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt),
                        drivetrain.followPathCommand(pathATo1).alongWith(intake.intakeNoteAutoCommand()),
                        drivetrain.followPathCommand(path1ToScore),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt),
                        drivetrain.followPathCommand(pathScoreTo2).alongWith(intake.intakeNoteAutoCommand()),
                        drivetrain.followPathCommand(path2ToScore),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt),
                        drivetrain.followPathCommand(pathScoreTo3).alongWith(intake.intakeNoteAutoCommand()),
                        drivetrain.followPathCommand(path3ToScore),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt)),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose));

        return new AutoRoutine("BA123", command,
                List.of(pathStartToB, pathBToA, pathATo1, path1ToScore, pathScoreTo2, path2ToScore, pathScoreTo3,
                        path3ToScore),
                new Pose2d(startingPose.getX(), startingPose.getY(), new Rotation2d(Math.PI)));
    }

    public static AutoRoutine autoCBA123(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt) {
        PathPlannerPath pathStartToC = PathPlannerPath.fromChoreoTrajectory("CBA123.1");
        PathPlannerPath pathCToB = PathPlannerPath.fromChoreoTrajectory("CBA123.2");
        PathPlannerPath pathBToA = PathPlannerPath.fromChoreoTrajectory("CBA123.3");
        PathPlannerPath pathATo1 = PathPlannerPath.fromChoreoTrajectory("CBA123.4");
        PathPlannerPath path1ToScore = PathPlannerPath.fromChoreoTrajectory("CBA123.5");
        PathPlannerPath pathScoreTo2 = PathPlannerPath.fromChoreoTrajectory("CBA123.6");
        PathPlannerPath path2ToScore = PathPlannerPath.fromChoreoTrajectory("CBA123.7");
        PathPlannerPath pathScoreTo3 = PathPlannerPath.fromChoreoTrajectory("CBA123.8");
        PathPlannerPath path3ToScore = PathPlannerPath.fromChoreoTrajectory("CBA123.9");

        Pose2d startingPose = pathStartToC.getPreviewStartingHolonomicPose();

        Command command = Commands.parallel(
                Commands.sequence(
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt)
                                .alongWith(intake.moveToPositionCommand(() -> IntakePosition.GROUND)),
                        drivetrain.followPathCommand(pathStartToC).alongWith(intake.intakeNoteAutoCommand()),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt),
                        drivetrain.followPathCommand(pathCToB).alongWith(intake.intakeNoteAutoCommand()),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt),
                        drivetrain.followPathCommand(pathBToA).alongWith(intake.intakeNoteAutoCommand()),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt),
                        drivetrain.followPathCommand(pathATo1).alongWith(intake.intakeNoteAutoCommand()),
                        drivetrain.followPathCommand(path1ToScore),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt),
                        drivetrain.followPathCommand(pathScoreTo2).alongWith(intake.intakeNoteAutoCommand()),
                        drivetrain.followPathCommand(path2ToScore),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt),
                        drivetrain.followPathCommand(pathScoreTo3).alongWith(intake.intakeNoteAutoCommand()),
                        drivetrain.followPathCommand(path3ToScore),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt)),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose));

        return new AutoRoutine("CBA123", command,
                List.of(pathStartToC, pathCToB, pathBToA, pathATo1, path1ToScore, pathScoreTo2, path2ToScore,
                        pathScoreTo3,
                        path3ToScore),
                new Pose2d(startingPose.getX(), startingPose.getY(), new Rotation2d(2.62276)));
    }

    public static AutoRoutine auto453(Drivetrain drivetrain, Intake intake, Shooter shooter, ShooterTilt shooterTilt) {
        PathPlannerPath pathStartTo4 = PathPlannerPath.fromChoreoTrajectory("453.1");
        PathPlannerPath path4ToScore = PathPlannerPath.fromChoreoTrajectory("453.2");
        PathPlannerPath pathScoreTo5 = PathPlannerPath.fromChoreoTrajectory("453.3");
        PathPlannerPath path5ToScore = PathPlannerPath.fromChoreoTrajectory("453.4");
        PathPlannerPath pathScoreTo3 = PathPlannerPath.fromChoreoTrajectory("453.5");
        PathPlannerPath path3ToScore = PathPlannerPath.fromChoreoTrajectory("453.6");

        Pose2d startingPose = pathStartTo4.getPreviewStartingHolonomicPose();

        Command command = Commands.parallel(
                Commands.sequence(
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt)
                                .alongWith(intake.moveToPositionCommand(() -> IntakePosition.GROUND)),
                        drivetrain.followPathCommand(pathStartTo4).alongWith(intake.intakeNoteAutoCommand()),
                        drivetrain.followPathCommand(path4ToScore),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt),
                        drivetrain.followPathCommand(pathScoreTo5).alongWith(intake.intakeNoteAutoCommand()),
                        drivetrain.followPathCommand(path5ToScore),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt),
                        drivetrain.followPathCommand(pathScoreTo3).alongWith(intake.intakeNoteAutoCommand()),
                        drivetrain.followPathCommand(path3ToScore),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt)),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose));

        return new AutoRoutine("453", command,
                List.of(pathStartTo4, path4ToScore, pathScoreTo5, path5ToScore, pathScoreTo3, path3ToScore),
                new Pose2d(startingPose.getX(), startingPose.getY(), new Rotation2d(1.986288)));
    }

    public static AutoRoutine auto1234(Drivetrain drivetrain, Intake intake, Shooter shooter, ShooterTilt shooterTilt) {
        PathPlannerPath pathStartTo1 = PathPlannerPath.fromChoreoTrajectory("1234.1");
        PathPlannerPath path1ToScore = PathPlannerPath.fromChoreoTrajectory("1234.2");
        PathPlannerPath pathScoreTo2 = PathPlannerPath.fromChoreoTrajectory("1234.3");
        PathPlannerPath path2ToScore = PathPlannerPath.fromChoreoTrajectory("1234.4");
        PathPlannerPath pathScoreTo3 = PathPlannerPath.fromChoreoTrajectory("1234.5");
        PathPlannerPath path3ToScore = PathPlannerPath.fromChoreoTrajectory("1234.6");
        PathPlannerPath pathScoreTo4 = PathPlannerPath.fromChoreoTrajectory("1234.7");
        PathPlannerPath path4ToScore = PathPlannerPath.fromChoreoTrajectory("1234.8");

        Pose2d startingPose = pathStartTo1.getPreviewStartingHolonomicPose();

        Command command = Commands.parallel(
                Commands.sequence(
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt)
                                .alongWith(intake.moveToPositionCommand(() -> IntakePosition.GROUND)),
                        drivetrain.followPathCommand(pathStartTo1).alongWith(intake.intakeNoteAutoCommand()),
                        drivetrain.followPathCommand(path1ToScore),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt),
                        drivetrain.followPathCommand(pathScoreTo2).alongWith(intake.intakeNoteAutoCommand()),
                        drivetrain.followPathCommand(path2ToScore),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt),
                        drivetrain.followPathCommand(pathScoreTo3).alongWith(intake.intakeNoteAutoCommand()),
                        drivetrain.followPathCommand(path3ToScore),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt),
                        drivetrain.followPathCommand(pathScoreTo4).alongWith(intake.intakeNoteAutoCommand()),
                        drivetrain.followPathCommand(path4ToScore),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt)

                ),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose));

        return new AutoRoutine("1234", command,
                List.of(pathStartTo1, path1ToScore, pathScoreTo2, path2ToScore, pathScoreTo3, path3ToScore,
                        pathScoreTo4, path4ToScore),
                new Pose2d(startingPose.getX(), startingPose.getY(), new Rotation2d(-2.32049)));
    }

    public static AutoRoutine autoBAC(Drivetrain drivetrain, Intake intake, Shooter shooter, ShooterTilt shooterTilt) {
        PathPlannerPath pathStartToB = PathPlannerPath.fromChoreoTrajectory("BAC.1");
        PathPlannerPath pathBToC = PathPlannerPath.fromChoreoTrajectory("BAC.2");
        PathPlannerPath pathCToA = PathPlannerPath.fromChoreoTrajectory("BAC.3");

        Pose2d startingPose = pathStartToB.getPreviewStartingHolonomicPose();

        Command command = Commands.parallel(
                Commands.sequence(
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt)
                                .alongWith(intake.moveToPositionCommand(() -> IntakePosition.GROUND)),
                        drivetrain.followPathCommand(pathStartToB).alongWith(intake.intakeNoteAutoCommand()),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt),
                        drivetrain.followPathCommand(pathBToC).alongWith(intake.intakeNoteAutoCommand()),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt),
                        drivetrain.followPathCommand(pathCToA).alongWith(intake.intakeNoteAutoCommand()),
                        RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt)),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose));

        return new AutoRoutine("BAC", command,
                List.of(pathStartToB, pathBToC, pathCToA),
                new Pose2d(startingPose.getX(), startingPose.getY(), new Rotation2d(Math.PI)));
    }
}
