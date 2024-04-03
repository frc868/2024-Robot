package frc.robot;

import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.SendableLogger;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.NoteLift;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTilt;

public class NTCommands {
    public static void configureNTCommands(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt, Climber climber, NoteLift noteLift, LEDs leds) {
        LoggingManager.getInstance().addGroup(new LogGroup(
                // new SendableLogger("commands/leds", "chase",
                // leds.requestChaseCommand()),
                // new SendableLogger("commands/leds", "wave",
                // leds.requestWaveCommand()),
                // new SendableLogger("commands/leds", "red",
                // leds.requestFlashingRedCommand()),
                // new SendableLogger("commands/leds", "breathe",
                // leds.requestBreatheCommand()),
                // new SendableLogger("commands/leds", "fire",
                // leds.requestFireCommand()),
                new SendableLogger("commands/intake", "resetPosition",
                        intake.resetPositionCommand().ignoringDisable(true)),
                new SendableLogger("commands/intake", "intakeBeam",
                        intake.simTriggerIntakeBeamCommand().ignoringDisable(true)),
                new SendableLogger("commands/intake", "shooterFarBeam",
                        intake.simTriggerShooterFarBeamCommand().ignoringDisable(true)),
                new SendableLogger("commands/intake", "shooterCloseBeam",
                        intake.simTriggerShooterCloseBeamCommand().ignoringDisable(true)),
                new SendableLogger("commands", "initialize",
                Commands.sequence(
                        drivetrain.setInitializedCommand(true),
                        intake.setInitializedCommand(true),
                        shooterTilt.setInitializedCommand(true),
                        climber.setInitializedCommand(true),
                        noteLift.setInitializedCommand(true),
                        GlobalStates.INITIALIZED.enableCommand()).ignoringDisable(true)),
                new SendableLogger("commands/shooterTilt", "resetPosition",
                        shooterTilt.resetPositionCommand().ignoringDisable(true)),
                new SendableLogger("commands/climber", "resetPosition",
                        climber.resetPositionCommand().ignoringDisable(true)),
                new SendableLogger("commands/noteLift", "resetPosition",
                        noteLift.resetPositionCommand().ignoringDisable(true))));
    }

}
