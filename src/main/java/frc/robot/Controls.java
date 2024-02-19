package frc.robot;

import com.techhounds.houndutil.houndlib.oi.CommandVirpilJoystick;
import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive.DriveMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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

        joystick.centerBottomHatButton().onTrue(drivetrain.wheelLockCommand());

        joystick.centerTopHatUp().whileTrue(intake.moveToPositionCommand(() -> IntakePosition.STOW));
        joystick.centerTopHatLeft().whileTrue(intake.moveToPositionCommand(() -> IntakePosition.AMP));
        joystick.centerTopHatDown().whileTrue(intake.moveToPositionCommand(() -> IntakePosition.GROUND));

        joystick.triggerSoftPress()
                .whileTrue(RobotCommands.targetSpeakerCommand(drivetrain, shooter, shooterTilt));
        joystick.triggerHardPress().whileTrue(RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt));

        joystick.blackThumbButton().whileTrue(intake.intakeNoteCommand())
                .onFalse(intake.moveToPositionCommand(() -> IntakePosition.STOW));

        joystick.centerTopHatButton().whileTrue(intake.intakeFromSourceCommand())
                .onFalse(intake.moveToPositionCommand(() -> IntakePosition.STOW));

        joystick.redButton().whileTrue(intake.ampScoreCommand(() -> joystick.getHID().getPinkieButton())
                .alongWith(shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.AMP_EJECT)));
    }

    public static void configureTestingControl(int port, Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt, Climber climber, NoteLift noteLift) {
        CommandXboxController controller = new CommandXboxController(port);

        controller.povUp().onTrue(shooterTilt.movePositionDeltaCommand(() -> 0.01));
        controller.povDown().onTrue(shooterTilt.movePositionDeltaCommand(() -> -0.01));
        controller.povRight().onTrue(Commands.runOnce(() -> Constants.Shooter.SHOOTING_RPS += 1));
        controller.povLeft().onTrue(Commands.runOnce(() -> Constants.Shooter.SHOOTING_RPS -= 1));

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

        // controller.x().whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
        // controller.y().whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));
        // controller.a().whileTrue(shooter.sysIdDynamic(Direction.kForward));
        // controller.b().whileTrue(shooter.sysIdDynamic(Direction.kReverse));
        // controller.a().whileTrue(noteLift.moveToPositionCommand(() ->
        // NoteLiftPosition.BOTTOM));
        // controller.x().whileTrue(noteLift.moveToPositionCommand(() ->
        // NoteLiftPosition.STOW));
        // controller.y().whileTrue(noteLift.moveToPositionCommand(() ->
        // NoteLiftPosition.TOP));

        controller.a().whileTrue(RobotCommands.intakeToNoteLift(shooter, shooterTilt,
                noteLift));
        controller.b().whileTrue(RobotCommands.prepareClimb(intake, shooter,
                shooterTilt, climber, noteLift));
        controller.x().whileTrue(climber.climbToBottomCommand());
        controller.y().whileTrue(noteLift.scoreNoteCommand());

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
