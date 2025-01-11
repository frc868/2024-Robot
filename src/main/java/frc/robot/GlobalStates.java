package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Overall robot states that are shared across every subsystem. These are used
 * to control overrides that should affect the entire robot.
 */
public enum GlobalStates {
    /** Whether the robot has been fully initialized. */
    INITIALIZED(RobotBase.isReal() ? false : true),
    /**
     * Whether the {@link atGoal} method of all subsystems should be overridden (to
     * prevent a mechanism that cannot reach its goal for mechanical reasons from
     * hanging automated features, i.e. shooter wheels can't get fully up to speed
     * because of a bent shaft, but you can still shoot in the speaker)
     */
    AT_GOAL_OVERRIDE(false),
    /**
     * Whether all inter-subsystem safeties should be disabled.
     */
    INTER_SUBSYSTEM_SAFETIES_DISABLED(false),
    /**
     * Whether the soft limits on each individual mechanism should be disabled,
     * allowing voltage inputs beyond the normal travel range of the mechanism.
     */
    MECH_LIMITS_DISABLED(false),
    /**
     * Whether the robot should disable vision tracking of the speaker and instead
     * target from up against the subwoofer.
     */
    SUBWOOFER_ONLY(false),
    /**
     * Whether the robot should disable vision tracking of the speaker and instead
     * target from up against the podium.
     */
    PODIUM_ONLY(false);

    private boolean isEnabled;

    private GlobalStates(boolean isEnabled) {
        this.isEnabled = isEnabled;
    }

    /**
     * Whether the state is enabled.
     * 
     * @return whether the state is enabled
     */
    public boolean enabled() {
        return this.isEnabled;
    }

    /**
     * Creates a command that enables this state.
     * 
     * @return a command that enables this state
     */
    public Command enableCommand() {
        return Commands.runOnce(() -> this.isEnabled = true).ignoringDisable(true);
    }

    /**
     * Creates a command that toggles this state.
     * 
     * @return a command that toggles this state
     */
    public Command toggleCommand() {
        return Commands.runOnce(() -> this.isEnabled = !this.isEnabled).ignoringDisable(true);
    }

    /**
     * Creates a command that disables this state.
     * 
     * @return a command that disables this state
     */
    public Command disableCommand() {
        return Commands.runOnce(() -> this.isEnabled = false).ignoringDisable(true);
    }
}