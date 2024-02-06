package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakingCommands {
    public static Command startAndRunIntakingCommand() {
        return Commands.sequence();
    }

}
