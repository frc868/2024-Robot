package frc.robot;

import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.loggers.SendableLogger;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.NoteLift;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTilt;

public class NTCommands {
    public static void configureNTCommands(Drivetrain drivetrain, Intake intake, Shooter shooter,
            ShooterTilt shooterTilt, Climber climber, NoteLift noteLift) {
        LoggingManager.getInstance().addGroup(new LogGroup(
                new SendableLogger("commands/intake", "resetPosition",
                        intake.resetPositionCommand().ignoringDisable(true)),
                new SendableLogger("commands/shooterTilt", "resetPosition",
                        shooterTilt.resetPositionCommand().ignoringDisable(true)),
                new SendableLogger("commands/climber", "resetPosition",
                        climber.resetPositionCommand().ignoringDisable(true)),
                new SendableLogger("commands/noteLift", "resetPosition",
                        noteLift.resetPositionCommand().ignoringDisable(true))));
    }

}
