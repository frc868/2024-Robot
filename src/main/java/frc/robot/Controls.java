package frc.robot;

import com.techhounds.houndutil.houndlib.oi.CommandVirpilJoystick;
import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive.DriveMode;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Intake.IntakePosition;
import frc.robot.Constants.ShooterTilt.ShooterTiltPosition;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteLift;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTilt;

public class Controls {
    private static double speedMultiplier = 1.0;

    public static void configureDriverControl(int port, Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt, Climber climber, NoteLift noteLift) {
        CommandVirpilJoystick joystick = new CommandVirpilJoystick(port);

        drivetrain.setDefaultCommand(
                drivetrain.teleopDriveCommand(
                        () -> -joystick.getY() * speedMultiplier,
                        () -> -joystick.getX() * speedMultiplier,
                        () -> -joystick.getTwist() * speedMultiplier));

        new Trigger(() -> (Math.abs(joystick.getTwist()) > 0.05)).onTrue(drivetrain.disableControlledRotateCommand());

        joystick.stickButton().onTrue(drivetrain.resetGyroCommand());

        joystick.centerBottomHatUp()
                .whileTrue(drivetrain.controlledRotateCommand(() -> Math.toRadians(0),
                        DriveMode.FIELD_ORIENTED));
        joystick.centerBottomHatLeft()
                .whileTrue(drivetrain.controlledRotateCommand(() -> Math.toRadians(90),
                        DriveMode.FIELD_ORIENTED));
        joystick.centerBottomHatDown()
                .whileTrue(drivetrain.controlledRotateCommand(() -> Math.toRadians(180),
                        DriveMode.FIELD_ORIENTED));
        joystick.centerBottomHatRight()
                .whileTrue(drivetrain.controlledRotateCommand(() -> Math.toRadians(270),
                        DriveMode.FIELD_ORIENTED));

        // joystick.centerTopHatButton().whileTrue(
        // drivetrain.targetStageCommand2(
        // () -> -joystick.getY() * speedMultiplier,
        // () -> -joystick.getX() * speedMultiplier));

        joystick.triggerSoftPress().and(joystick.flipTriggerIn().negate())
                .whileTrue(RobotCommands.targetSpeakerOnTheMoveCommand(drivetrain, shooter,
                        shooterTilt));
        joystick.triggerHardPress().and(joystick.flipTriggerIn().negate())
                .whileTrue(RobotCommands.shootOnTheMoveCommand(drivetrain,
                        intake, shooter, shooterTilt));

        joystick.blackThumbButton()
                .whileTrue(intake.intakeNoteCommand()
                        .alongWith(shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.INTAKE).asProxy()))
                .onFalse(intake.moveToPositionCommand(() -> IntakePosition.STOW));

        joystick.centerTopHatButton().whileTrue(intake.intakeFromSourceCommand())
                .onFalse(intake.moveToPositionCommand(() -> IntakePosition.STOW));

        joystick.redButton().whileTrue(intake.ampPrepCommand()
                .alongWith(shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.AMP_EJECT)));
        joystick.pinkieButton().whileTrue(intake.ampScoreRollersCommand())
                .onFalse(intake.moveToPositionCommand(() -> IntakePosition.STOW));

        joystick.bottomHatButton().or(joystick.bottomHatLeft())
                .onTrue(RobotCommands.intakeToNoteLift(shooter, shooterTilt,
                        noteLift));
        joystick.flipTriggerIn()
                .whileTrue(RobotCommands.prepareClimb(() -> -joystick.getY(), () -> -joystick.getX(), drivetrain,
                        intake, shooter,
                        shooterTilt, climber, noteLift));
        joystick.flipTriggerIn().and(joystick.triggerSoftPress().or(joystick.triggerHardPress()))
                .whileTrue(climber.climbToBottomCommand().andThen(noteLift.scoreNoteCommand()));
    }

    public static void configureTestingControl(int port, Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt, Climber climber, NoteLift noteLift) {
        CommandXboxController controller = new CommandXboxController(port);

        controller.x().toggleOnTrue(intake.setOverridenSpeedCommand(() -> -controller.getLeftY() * 0.25)
                .finallyDo(intake.resetControllersCommand()::schedule));
        controller.y().toggleOnTrue(shooterTilt.setOverridenSpeedCommand(() -> -controller.getRightY() * 0.75)
                .finallyDo(shooterTilt.resetControllersCommand()::schedule));
        controller.a().toggleOnTrue(climber.setOverridenSpeedCommand(() -> -controller.getLeftY() * 0.75)
                .finallyDo(climber.resetControllersCommand()::schedule));
        controller.b().toggleOnTrue(noteLift.setOverridenSpeedCommand(() -> -controller.getRightY() * 0.75)
                .finallyDo(noteLift.resetControllersCommand()::schedule));

        controller.x().and(controller.povDown()).whileTrue(intake.moveToPositionCommand(() -> IntakePosition.GROUND));
        controller.x().and(controller.povUp()).whileTrue(intake.moveToPositionCommand(() -> IntakePosition.STOW));
        controller.x().and(controller.povLeft()).whileTrue(intake.moveToPositionCommand(() -> IntakePosition.AMP));
        controller.x().and(controller.povRight()).whileTrue(intake.moveToPositionCommand(() -> IntakePosition.SOURCE));

        controller.povDown().whileTrue(RobotCommands.resetClimb(intake, shooter,
                shooterTilt, climber, noteLift));
        controller.povUp().whileTrue(RobotCommands.deClimb(intake, shooter,
                shooterTilt, climber, noteLift));
        controller.povRight()
                .whileTrue(RobotCommands.moveToHomeCommand(intake, shooter, shooterTilt,
                        climber, noteLift));

        // controller.a()
        // .whileTrue(drivetrain.targetStageCommand2(() -> -controller.getLeftY(), () ->
        // -controller.getLeftX()));
        // controller.povLeft().whileTrue(climber.setOverridenSpeedCommand(() ->
        // -controller.getLeftY()));
        // controller.povRight().whileTrue(shooterTilt.moveToPositionCommand(() ->
        // ShooterTiltPosition.CLIMB));

        // controller.x().whileTrue(shooter.spinAtVelocityCommand(() ->
        // Constants.Shooter.SHOOTING_RPS));
        // controller.y().whileTrue(intake.runRollersCommand());d

        // climber.setDefaultCommand();

        // noteLift.setDefaultCommand();

        // controller.x().toggleOnTrue(
        // Commands.parallel(
        // climber.setOverridenSpeedCommand(
        // () -> MathUtil.applyDeadband(-controller.getLeftY(), 0.08)),
        // noteLift.setOverridenSpeedCommand(
        // () -> {
        // double value = MathUtil.applyDeadband(-controller.getRightY(), 0.08);
        // if (value < 0) {
        // value *= 0.3;
        // }
        // return value;
        // })));
        // // controller.y().whileTrue();

        // controller.a().toggleOnTrue(Commands.run(shooter::stop, shooter));

        // controller.a().whileTrue(intake.moveToPositionCommand(() ->
        // IntakePosition.GROUND));
        // controller.y().whileTrue(intake.moveToPositionCommand(() ->
        // IntakePosition.STOW));
        // controller.x().whileTrue(RobotCommands.shootCommand(drivetrain, intake,
        // shooter, shooterTilt));
        // controller.b().whileTrue(RobotCommands.targetSpeakerCommand(drivetrain,
        // shooter, shooterTilt));

        // controller.a().whileTrue(noteLift.moveToPositionCommand(() ->
        // NoteLiftPosition.BOTTOM));
        // controller.x().whileTrue(noteLift.moveToPositionCommand(() ->
        // NoteLiftPosition.STOW));
        // controller.y().whileTrue(noteLift.moveToPositionCommand(() ->
        // NoteLiftPosition.TOP));

        // controller.y().whileTrue(noteLift.scoreNoteCommand());/

        // controller.a()
        // .whileTrue(drivetrain.targetStageCommand(() -> -controller.getLeftY(), () ->
        // -controller.getLeftX()));

        // controller.y().whileTrue(noteLift.moveToPositionCommand(() ->
        // NoteLiftPosition.TOP));
        // controller.b().whileTrue(noteLift.moveToPositionCommand(() ->
        // NoteLiftPosition.STOW));
        // controller.x().whileTrue(noteLift.moveToPositionCommand(() ->
        // NoteLiftPosition.CLIMB_PREP));
        // controller.a().whileTrue(noteLift.moveToPositionCommand(() ->
        // NoteLiftPosition.BOTTOM));
    }

}
