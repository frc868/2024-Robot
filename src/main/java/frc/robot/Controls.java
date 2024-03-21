package frc.robot;

import com.techhounds.houndutil.houndlib.oi.CommandVirpilJoystick;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Intake.IntakePosition;
import frc.robot.Constants.ShooterTilt.ShooterTiltPosition;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.NoteLift;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTilt;

public class Controls {
    public static void configureDriverControls(int port, Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt, Climber climber, NoteLift noteLift, LEDs leds) {
        CommandVirpilJoystick joystick = new CommandVirpilJoystick(port);

        drivetrain.setDefaultCommand(
                drivetrain.teleopDriveCommand(
                        () -> -joystick.getY(),
                        () -> -joystick.getX(),
                        () -> -joystick.getTwist()));

        new Trigger(() -> (Math.abs(joystick.getTwist()) > 0.05)).onTrue(drivetrain.disableControlledRotateCommand());

        joystick.stickButton().onTrue(drivetrain.resetGyroCommand());

        joystick.centerBottomHatUp()
                .whileTrue(drivetrain.controlledRotateCommand(() -> Math.toRadians(0)));
        joystick.centerBottomHatLeft()
                .whileTrue(drivetrain.controlledRotateCommand(() -> Math.toRadians(270)));
        joystick.centerBottomHatDown()
                .whileTrue(drivetrain.controlledRotateCommand(() -> Math.toRadians(180)));
        joystick.centerBottomHatRight()
                .whileTrue(drivetrain.controlledRotateCommand(() -> Math.toRadians(90)));

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

        joystick.redButton().whileTrue(RobotCommands.ampPrepIntakeCommand(intake, shooterTilt))
                .onFalse(intake.moveToPositionCommand(() -> IntakePosition.AMP));

        joystick.pinkieButton().whileTrue(intake.ampScoreRollersCommand())
                .onFalse(intake.moveToPositionCommand(() -> IntakePosition.STOW));

        joystick.bottomHatButton().or(joystick.bottomHatLeft())
                .onTrue(RobotCommands.intakeToNoteLift(shooter, shooterTilt,
                        noteLift, leds));
        joystick.flipTriggerIn()
                .whileTrue(RobotCommands.prepareClimb(() -> -joystick.getY(), () -> -joystick.getX(),
                        () -> -joystick.getTwist(), drivetrain,
                        intake, shooter,
                        shooterTilt, climber, noteLift));
        joystick.flipTriggerIn().and(joystick.triggerSoftPress().or(joystick.triggerHardPress()))
                .whileTrue(climber.climbToBottomCommand().andThen(noteLift.scoreNoteCommand()));
    }

    public static void configureOperatorControls(int port, Drivetrain drivetrain, Intake intake, Shooter shooter,
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

        controller.povDown().whileTrue(RobotCommands.resetClimb(intake, shooter,
                shooterTilt, climber, noteLift));
        controller.povUp().whileTrue(RobotCommands.deClimb(intake, shooter,
                shooterTilt, climber, noteLift));
        controller.povRight()
                .whileTrue(RobotCommands.moveToHomeCommand(intake, shooter, shooterTilt,
                        climber, noteLift));
        controller.povLeft()
                .whileTrue(RobotCommands.homeMechanismsCommand(intake, shooter, shooterTilt, climber, noteLift));
    }

    public static void configureOverridesControls(int port, Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt, Climber climber, NoteLift noteLift) {
        CommandGenericHID panel = new CommandGenericHID(port);

        panel.button(1)
                .onTrue(GlobalStates.INITIALIZED.enableCommand())
                .onFalse(GlobalStates.INITIALIZED.disableCommand());
        panel.button(2)
                .onTrue(GlobalStates.AT_GOAL_OVERRIDE.enableCommand())
                .onFalse(GlobalStates.AT_GOAL_OVERRIDE.disableCommand());
        panel.button(3)
                .onTrue(GlobalStates.INTER_SUBSYSTEM_SAFETIES_DISABLED.enableCommand())
                .onFalse(GlobalStates.INTER_SUBSYSTEM_SAFETIES_DISABLED.disableCommand());
        panel.button(4)
                .onTrue(GlobalStates.MECH_LIMITS_DISABLED.enableCommand())
                .onFalse(GlobalStates.MECH_LIMITS_DISABLED.disableCommand());
    }

    public static void configureTestingControls(int port, Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt, Climber climber, NoteLift noteLift) {
        CommandXboxController controller = new CommandXboxController(port);

        controller.x().and(controller.povUp()).whileTrue(intake.sysIdQuasistatic(Direction.kForward));
        controller.y().and(controller.povUp()).whileTrue(intake.sysIdQuasistatic(Direction.kReverse));
        controller.a().and(controller.povUp()).whileTrue(intake.sysIdDynamic(Direction.kForward));
        controller.b().and(controller.povUp()).whileTrue(intake.sysIdDynamic(Direction.kReverse));

        controller.x().and(controller.povLeft()).whileTrue(shooterTilt.sysIdQuasistatic(Direction.kForward));
        controller.y().and(controller.povLeft()).whileTrue(shooterTilt.sysIdQuasistatic(Direction.kReverse));
        controller.a().and(controller.povLeft()).whileTrue(shooterTilt.sysIdDynamic(Direction.kForward));
        controller.b().and(controller.povLeft()).whileTrue(shooterTilt.sysIdDynamic(Direction.kReverse));

        controller.x().and(controller.povRight()).whileTrue(climber.sysIdQuasistatic(Direction.kForward));
        controller.y().and(controller.povRight()).whileTrue(climber.sysIdQuasistatic(Direction.kReverse));
        controller.a().and(controller.povRight()).whileTrue(climber.sysIdDynamic(Direction.kForward));
        controller.b().and(controller.povRight()).whileTrue(climber.sysIdDynamic(Direction.kReverse));

        controller.x().and(controller.povDown()).whileTrue(noteLift.sysIdQuasistatic(Direction.kForward));
        controller.y().and(controller.povDown()).whileTrue(noteLift.sysIdQuasistatic(Direction.kReverse));
        controller.a().and(controller.povDown()).whileTrue(noteLift.sysIdDynamic(Direction.kForward));
        controller.b().and(controller.povDown()).whileTrue(noteLift.sysIdDynamic(Direction.kReverse));

        controller.x().and(controller.leftBumper()).whileTrue(drivetrain.sysIdDriveQuasistatic(Direction.kForward));
        controller.y().and(controller.leftBumper()).whileTrue(drivetrain.sysIdDriveQuasistatic(Direction.kReverse));
        controller.a().and(controller.leftBumper()).whileTrue(drivetrain.sysIdDriveDynamic(Direction.kForward));
        controller.b().and(controller.leftBumper()).whileTrue(drivetrain.sysIdDriveDynamic(Direction.kReverse));

        controller.x().and(controller.rightBumper()).whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
        controller.y().and(controller.rightBumper()).whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));
        controller.a().and(controller.rightBumper()).whileTrue(shooter.sysIdDynamic(Direction.kForward));
        controller.b().and(controller.rightBumper()).whileTrue(shooter.sysIdDynamic(Direction.kReverse));

    }

}
