package frc.robot;

import com.techhounds.houndutil.houndlib.oi.CommandVirpilJoystick;
import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive.DriveMode;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Intake.IntakePosition;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.NoteLift;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTilt;

public class Controls {
    private static double speedMultiplier = 1.0;

    public static void configureDriverControl(int port, Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt, Climber climber, NoteLift noteLift, LEDs leds) {
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

        joystick.triggerSoftPress().whileTrue(RobotCommands.targetSpeakerCommand(drivetrain, shooter, shooterTilt));
        joystick.triggerHardPress().onTrue(RobotCommands.shootCommand(drivetrain, intake, shooter, shooterTilt));

        joystick.blackThumbButton().whileTrue(intake.ampScoreCommand(() -> joystick.getHID().getPinkieButton()));
    }
}