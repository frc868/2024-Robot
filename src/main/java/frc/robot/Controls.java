package frc.robot;

import com.techhounds.houndutil.houndlib.oi.CommandVirpilJoystick;
import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive.DriveMode;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

        joystick.centerBottomHatButton().onTrue(drivetrain.wheelLockCommand());

        joystick.centerTopHatUp().whileTrue(intake.moveToPositionCommand(() -> IntakePosition.STOW));
        joystick.centerTopHatLeft().whileTrue(intake.moveToPositionCommand(() -> IntakePosition.AMP));
        joystick.centerTopHatDown().whileTrue(intake.moveToPositionCommand(() -> IntakePosition.GROUND));

        joystick.triggerSoftPress()
                .whileTrue(RobotCommands.targetFromSubwooferCommand(drivetrain, shooter, shooterTilt));
        joystick.triggerHardPress().whileTrue(RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt));

        joystick.blackThumbButton().whileTrue(intake.intakeNoteCommand())
                .onFalse(intake.moveToPositionCommand(() -> IntakePosition.STOW));

        joystick.redButton().whileTrue(intake.ampScoreCommand(() -> joystick.getHID().getPinkieButton())
                .alongWith(shooterTilt.moveToPositionCommand(() -> ShooterTiltPosition.AMP_EJECT)));
    }

    public static void configureTestingControl(int port, Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt, Climber climber, NoteLift noteLift) {
        CommandXboxController controller = new CommandXboxController(port);

        controller.povUp().onTrue(shooterTilt.movePositionDeltaCommand(() -> 0.02));
        controller.povDown().onTrue(shooterTilt.movePositionDeltaCommand(() -> -0.02));
        controller.povRight().onTrue(Commands.runOnce(() -> Constants.Shooter.SHOOTING_RPS += 1));
        controller.povLeft().onTrue(Commands.runOnce(() -> Constants.Shooter.SHOOTING_RPS -= 1));

        // controller.x().whileTrue(shooter.spinAtVelocityCommand(() ->
        // Constants.Shooter.SHOOTING_RPS));
        // controller.y().whileTrue(intake.runRollersCommand());

        controller.x().whileTrue(climber
                .setOverridenSpeedCommand(() -> controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()));
        controller.y().whileTrue(noteLift
                .setOverridenSpeedCommand(() -> controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()));

    }

}
