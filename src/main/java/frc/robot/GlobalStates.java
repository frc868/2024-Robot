package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public enum GlobalStates {
    // normal
    INITIALIZED(false);

    // coast
    // DRIVETRAIN_COAST(false),

    // MANUAL_MECH_CONTROL_MODE(false),
    // ABSOLUTE_ENCODERS(false),
    // SAFETIES_DISABLE(true),
    // SPEED_LIMITS_DISABLE(true),
    // MECH_LIMITS_DISABLE(true),
    // DRIVER_EMERGENCY_MODE(false);

    private boolean isEnabled;

    private GlobalStates(boolean isEnabled) {
        this.isEnabled = isEnabled;
    }

    public boolean enabled() {
        return this.isEnabled;
    }

    public Command enableCommand() {
        return Commands.runOnce(() -> this.isEnabled = true);
    }

    public Command disableCommand() {
        return Commands.runOnce(() -> this.isEnabled = false);
    }
}
