package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public enum GlobalStates {
    INITIALIZED(RobotBase.isReal() ? false : true),
    AT_GOAL_OVERRIDE(false),
    INTER_SUBSYSTEM_SAFETIES_DISABLED(false),
    MECH_LIMITS_DISABLED(false),
    BEAMS_BYPASSED(false),
    SUBWOOFER_ONLY(false),
    QUICK_CLIMB(false),
    PODIUM_ONLY(false);

    private boolean isEnabled;

    private GlobalStates(boolean isEnabled) {
        this.isEnabled = isEnabled;
    }

    public boolean enabled() {
        return this.isEnabled;
    }

    public Command enableCommand() {
        return Commands.runOnce(() -> this.isEnabled = true).ignoringDisable(true);
    }

    public Command toggleCommand() {
        return Commands.runOnce(() -> this.isEnabled = !this.isEnabled).ignoringDisable(true);
    }

    public Command disableCommand() {
        return Commands.runOnce(() -> this.isEnabled = false).ignoringDisable(true);
    }
}