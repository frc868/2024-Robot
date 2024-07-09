package frc.robot;

import static frc.robot.Constants.Drivetrain.DEMO_SPEED;
import static frc.robot.Constants.Shooter.DEMO_RPS;
import static frc.robot.Constants.Shooter.PODIUM_RPS;
import static frc.robot.Constants.ShooterTilt.DEMO_ANGLE;

import com.techhounds.houndutil.houndlib.oi.CommandVirpilJoystick;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Climber.ClimberPosition;
import frc.robot.Constants.Intake.IntakePosition;
import frc.robot.Constants.ShooterTilt.ShooterTiltPosition;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTilt;
import frc.robot.subsystems.LEDs.LEDState;

public class Controls {
    public static void configureDriverControls(int port, Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt, Climber climber, LEDs leds) {
        CommandVirpilJoystick joystick = new CommandVirpilJoystick(port);

        drivetrain.setDefaultCommand(
                drivetrain.teleopDriveCommand(
                        () -> -joystick.getY() * DEMO_SPEED.get(),
                        () -> -joystick.getX() * DEMO_SPEED.get(),
                        () -> -joystick.getTwist() * DEMO_SPEED.get()));

        new Trigger(() -> (Math.abs(joystick.getTwist()) > 0.05))
                .whileTrue(drivetrain.disableControlledRotateCommand());

        joystick.stickButton().onTrue(drivetrain.resetGyroCommand());

        joystick.centerBottomHatUp()
                .whileTrue(drivetrain.controlledRotateCommand(() -> Math.toRadians(0)));
        joystick.centerBottomHatLeft()
                .whileTrue(drivetrain.controlledRotateCommand(() -> Math.toRadians(270)));
        joystick.centerBottomHatDown()
                .whileTrue(drivetrain.controlledRotateCommand(() -> Math.toRadians(180)));
        joystick.centerBottomHatRight()
                .whileTrue(drivetrain.controlledRotateCommand(() -> Math.toRadians(90)));

        joystick.blackThumbButton()
                .whileTrue(intake.intakeNoteCommand()
                        .alongWith(shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.INTAKE).asProxy()))
                .onFalse(intake.moveToPositionCommand(() -> IntakePosition.STOW));
        joystick.centerTopHatButton().whileTrue(
                Commands.parallel(
                        intake.intakeFromSourceCommand(),
                        leds.requestStateCommand(LEDState.FLASHING_AQUA)))
                .onFalse(intake.moveToPositionCommand(() -> IntakePosition.STOW));

        // joystick.centerTopHatUp().whileTrue(
        // climber.moveUpCommand()
        // .deadlineWith(shooterTilt.moveToPositionCommand(() ->
        // ShooterTiltPosition.PODIUM)));
        // joystick.centerTopHatDown().whileTrue(
        // climber.moveDownCommand()
        // .deadlineWith(shooterTilt.moveToPositionCommand(() ->
        // ShooterTiltPosition.PODIUM)));

        joystick.redButton().whileTrue(RobotCommands.ampPrepIntakeCommand(intake, shooterTilt))
                .onFalse(intake.moveToPositionCommand(() -> IntakePosition.AMP));

        joystick.pinkieButton().whileTrue(intake.ampScoreRollersCommand())
                .onFalse(intake.moveToPositionCommand(() -> IntakePosition.STOW));

        joystick.triggerSoftPress().and(joystick.flipTriggerIn().negate()).whileTrue(
                Commands.parallel(
                        shooterTilt.moveToArbitraryPositionCommand(() -> DEMO_ANGLE.get()).asProxy(),
                        shooter.spinAtVelocityCommand(() -> DEMO_RPS.get()).asProxy()));
        joystick.triggerHardPress().and(joystick.flipTriggerIn().negate()).whileTrue(intake.runRollersCommand());

        joystick.flipTriggerIn().and(joystick.triggerSoftPress()).whileTrue(
                RobotCommands.targetPassCommand(drivetrain, shooter, shooterTilt));
        joystick.flipTriggerIn().and(joystick.triggerHardPress()).whileTrue(intake.runRollersCommand());

        joystick.topRightHatUp()
                .onTrue(GlobalStates.PODIUM_ONLY.enableCommand().andThen(GlobalStates.SUBWOOFER_ONLY.disableCommand()));
        joystick.topRightHatDown()
                .onTrue(GlobalStates.SUBWOOFER_ONLY.enableCommand().andThen(GlobalStates.PODIUM_ONLY.disableCommand()));
        joystick.topRightHatButton()
                .onTrue(GlobalStates.SUBWOOFER_ONLY.disableCommand()
                        .andThen(GlobalStates.PODIUM_ONLY.disableCommand()));

    }

    public static void configureOperatorControls(int port, Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt, Climber climber) {
        CommandXboxController controller = new CommandXboxController(port);

        controller.x().toggleOnTrue(intake.setOverridenSpeedCommand(() -> -controller.getLeftY() * 0.25)
                .finallyDo(intake.resetControllersCommand()::schedule));
        controller.y().toggleOnTrue(shooterTilt.setOverridenSpeedCommand(() -> -controller.getRightY() * 0.75)
                .finallyDo(shooterTilt.resetControllersCommand()::schedule));
        controller.a().toggleOnTrue(climber.setOverridenSpeedCommand(() -> -controller.getLeftY() * 0.75)
                .finallyDo(climber.resetControllersCommand()::schedule));

        controller.leftBumper().whileTrue(shooter.spinAtVelocityCommand(() -> PODIUM_RPS));
        controller.rightBumper().whileTrue(intake.runRollersCommand());

        controller.start().onTrue(GlobalStates.AT_GOAL_OVERRIDE.enableCommand())
                .onFalse(GlobalStates.AT_GOAL_OVERRIDE.disableCommand());
        controller.back().whileTrue(intake.reverseRollersCommand());

        controller.povDown().whileTrue(RobotCommands.resetClimb(intake, shooter,
                shooterTilt, climber));
        controller.povUp().whileTrue(RobotCommands.deClimb(intake, shooter,
                shooterTilt, climber).finallyDo(climber.resetControllersCommand()::schedule));
        controller.povRight()
                .whileTrue(RobotCommands.moveToHomeCommand(intake, shooter, shooterTilt,
                        climber));
        controller.povLeft().onTrue(
                Commands.sequence(
                        drivetrain.setInitializedCommand(true),
                        intake.setInitializedCommand(true),
                        shooterTilt.setInitializedCommand(true),
                        climber.setInitializedCommand(true),
                        GlobalStates.INITIALIZED.enableCommand()).ignoringDisable(true));
    }

    public static void configureOverridesControls(int port, Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt, Climber climber) {
        CommandXboxController controller = new CommandXboxController(port);

        controller.x().onTrue(GlobalStates.INTER_SUBSYSTEM_SAFETIES_DISABLED.enableCommand());
        controller.y().onTrue(GlobalStates.MECH_LIMITS_DISABLED.enableCommand());
        controller.b().onTrue(
                GlobalStates.MECH_LIMITS_DISABLED.disableCommand()
                        .andThen(GlobalStates.INTER_SUBSYSTEM_SAFETIES_DISABLED.disableCommand()));

        controller.a().onTrue(shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.SUBWOOFER).asProxy()
                .andThen(climber.moveToPositionCommand(() -> ClimberPosition.STOW).asProxy()));
    }

    public static void configureTestingControls(int port, Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt, Climber climber) {
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
