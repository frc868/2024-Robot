package frc.robot;

import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.LogGroup;
import com.techhounds.houndutil.houndlog.loggers.SendableLogItem;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTilt;

/**
 * Contains commands that are sent over NetworkTables (mainly for simulation,
 * but some are applicable to real robot).
 */
public class NTCommands {
    /**
     * Adds commands to NetworkTables for mechanism resetting, initialization, and
     * simulation triggering of beam breaks.
     * 
     * @param drivetrain  the drivetrain
     * @param intake      the intake
     * @param shooter     the shooter
     * @param shooterTilt the shooter tilt
     * @param climber     the climber
     * @param leds        the LEDs
     */
    public static void configureNTCommands(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt, Climber climber, LEDs leds) {
        LoggingManager.getInstance().addGroup(new LogGroup("commands",
                new LogGroup("intake",
                        new SendableLogItem("resetPosition",
                                intake.resetPositionCommand().ignoringDisable(true)),
                        new SendableLogItem("intakeBeam",
                                intake.simTriggerIntakeBeamCommand().ignoringDisable(true)),
                        new SendableLogItem("shooterFarBeam",
                                intake.simTriggerShooterFarBeamCommand().ignoringDisable(true)),
                        new SendableLogItem("shooterCloseBeam",
                                intake.simTriggerShooterCloseBeamCommand().ignoringDisable(true))),
                new SendableLogItem("initialize",
                        Commands.sequence(
                                drivetrain.setInitializedCommand(true),
                                intake.setInitializedCommand(true),
                                shooterTilt.setInitializedCommand(true),
                                climber.setInitializedCommand(true),
                                GlobalStates.INITIALIZED.enableCommand()).ignoringDisable(true)),
                new SendableLogItem("shooterTilt/resetPosition",
                        shooterTilt.resetPositionCommand().ignoringDisable(true)),
                new SendableLogItem("climber/resetPosition",
                        climber.resetPositionCommand().ignoringDisable(true))));
    }

}
