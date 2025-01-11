package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDs.LEDSection;

import static frc.robot.Constants.LEDs.*;
import static com.techhounds.houndutil.houndlib.leds.LEDPatterns.*;

/**
 * The LED subsystem, which controls the state of the LEDs by superimposing
 * requested LED states and continuously updates the LED's buffer. Other classes
 * can request specific LED states to be active, and they will be applied in
 * priority order.
 * 
 * @author dr
 */
@LoggedObject
public class LEDs extends SubsystemBase {
    /** The LEDs. */
    private AddressableLED leds = new AddressableLED(9);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(LENGTH);
    /** Notifier thread for displaying when the robot code is initializing. */
    private final Notifier loadingNotifier;

    private ArrayList<LEDState> currentStates = new ArrayList<LEDState>();

    /**
     * An enum of all possible LED states.
     */
    public enum LEDState {
        OFF(solid(Color.kBlack, LEDSection.ALL)),
        INTER_SUBSYSTEM_SAFETIES_DISABLED(
                breathe(Color.kOrange, 0.5, 0, 255, LEDSection.ELEVATOR_LEFT_TOP),
                breathe(Color.kOrange, 0.5, 0, 255, LEDSection.ELEVATOR_RIGHT_TOP)),
        MECH_LIMITS_DISABLED(
                breathe(Color.kOrange, 0.5, 0, 255, LEDSection.ELEVATOR_LEFT_BOTTOM),
                breathe(Color.kOrange, 0.5, 0, 255, LEDSection.ELEVATOR_RIGHT_BOTTOM)),

        RED_BREATHE(breathe(Color.kRed, 3, 0, 255, LEDSection.ALL)),
        NOTE_LIFT_UNINITIALIZED(
                breathe(Color.kRed, 3, 0, 255, LEDSection.ELEVATOR_LEFT_TOP),
                breathe(Color.kRed, 3, 0, 255, LEDSection.ELEVATOR_RIGHT_TOP)),
        CLIMBER_UNINITIALIZED(
                breathe(Color.kRed, 3, 0, 255, LEDSection.ELEVATOR_LEFT_BOTTOM),
                breathe(Color.kRed, 3, 0, 255, LEDSection.ELEVATOR_RIGHT_BOTTOM)),
        CLIMBER_BAD(
                flash(Color.kRed, 0.5, LEDSection.ELEVATOR_LEFT_BOTTOM),
                flash(Color.kRed, 0.5, LEDSection.ELEVATOR_RIGHT_BOTTOM)),
        INTAKE_UNINITIALIZED(
                breathe(Color.kRed, 3, 0, 255, LEDSection.SHOOTER_LEFT_BOTTOM),
                breathe(Color.kRed, 3, 0, 255, LEDSection.SHOOTER_RIGHT_BOTTOM)),
        SHOOTER_TILT_UNINITIALIZED(
                breathe(Color.kRed, 3, 0, 255, LEDSection.SHOOTER_LEFT_TOP),
                breathe(Color.kRed, 3, 0, 255, LEDSection.SHOOTER_RIGHT_TOP)),
        DRIVETRAIN_GYRO_UNINITIALIZED(
                breathe(Color.kRed, 3, 0, 255, LEDSection.SHOOTER_TOP)),
        INITIALIZATION_BLACK_BACKGROUND(solid(Color.kBlack, LEDSection.ALL)),
        INITIALIZED_CONFIRM(breathe(Color.kGreen, 2, 0, 255, LEDSection.ALL)),

        FLASHING_WHITE(flash(Color.kWhite, 0.5, LEDSection.ALL)),
        QUICK_CLIMB(
                breathe(Color.kBlue, 0.5, 0, 255, LEDSection.ELEVATOR_LEFT),
                breathe(Color.kBlue, 0.5, 0, 255, LEDSection.ELEVATOR_RIGHT)),
        SUBWOOFER_ONLY(flash(Color.kYellow, 0.3, LEDSection.SHOOTER_TOP)),
        PODIUM_ONLY(flash(Color.kBlue, 0.3, LEDSection.SHOOTER_TOP)),
        SOLID_BLUE(solid(Color.kBlue, LEDSection.ALL)),
        SOLID_GREEN(solid(Color.kGreen, LEDSection.ALL)),
        FLASHING_AQUA(flash(Color.kAqua, 0.5, LEDSection.ALL)),

        // decorative
        PURPLE_WAVE(
                wave(new Color("#9000DD"), 40, 10, 50, 255, LEDSection.SHOOTER_LEFT_EXT),
                wave(new Color("#9000DD"), 40, 10, 50, 255, LEDSection.SHOOTER_RIGHT_EXT)),
        GOLD_WAVE(
                wave(new Color("#FBBF05"), 25, 10, 50, 255, LEDSection.SHOOTER_LEFT_EXT),
                wave(new Color("#FBBF05"), 25, 10, 50, 255, LEDSection.SHOOTER_RIGHT_EXT)),
        RAINBOW_WAVE(
                waveRainbow(1, 30, 20, 100, 255, LEDSection.SHOOTER_LEFT_EXT),
                waveRainbow(1, 30, 20, 100, 255, LEDSection.SHOOTER_RIGHT_EXT)),
        PURPLE_FIRE(
                fire2012Palette(0.8, 0.4,
                        List.of(Color.kBlack, new Color("#ad3fe8"), new Color("#9000DD"), new Color("#400063")),
                        LEDSection.ELEVATOR_LEFT),
                fire2012Palette(0.8, 0.4,
                        List.of(Color.kBlack, new Color("#ad3fe8"), new Color("#9000DD"), new Color("#400063")),
                        LEDSection.ELEVATOR_RIGHT)),
        BLUE_FIRE(
                fire2012Palette(0.8, 0.4,
                        List.of(Color.kBlack, new Color("#3f4ae8"), new Color("#0008de"), new Color("#001fbb")),
                        LEDSection.ELEVATOR_LEFT),
                fire2012Palette(0.8, 0.4,
                        List.of(Color.kBlack, new Color("#3f4ae8"), new Color("#0008de"), new Color("#001fbb")),
                        LEDSection.ELEVATOR_RIGHT)),
        RAINBOW_FIRE(
                fire2012Rainbow(0.8, 0.4, 1, LEDSection.ELEVATOR_LEFT),
                fire2012Rainbow(0.8, 0.4, 1, LEDSection.ELEVATOR_RIGHT)),
        RAINBOW(rainbow(255, 3, LEDSection.SHOOTER_LEFT_EXT),
                rainbow(255, 3, LEDSection.SHOOTER_RIGHT_EXT),
                rainbow(255, 3, LEDSection.ELEVATOR_LEFT),
                rainbow(255, 3, LEDSection.ELEVATOR_RIGHT)),
        RED_FIRE(
                fire2012Palette(0.5, 0.25,
                        List.of(Color.kBlack, Color.kRed, Color.kOrangeRed, Color.kOrange, Color.kWhite),
                        LEDSection.ELEVATOR_LEFT),
                fire2012Palette(0.5, 0.25,
                        List.of(Color.kBlack, Color.kRed, Color.kOrangeRed, Color.kOrange, Color.kWhite),
                        LEDSection.ELEVATOR_RIGHT),
                fire2012Palette(0.5, 0.25,
                        List.of(Color.kBlack, Color.kRed, Color.kOrangeRed, Color.kOrange, Color.kWhite),
                        LEDSection.SHOOTER_LEFT),
                fire2012Palette(0.5, 0.25,
                        List.of(Color.kBlack, Color.kRed, Color.kOrangeRed, Color.kOrange, Color.kWhite),
                        LEDSection.SHOOTER_RIGHT),
                fire2012Palette(0.5, 0.25,
                        List.of(Color.kBlack, Color.kRed, Color.kOrangeRed, Color.kOrange, Color.kWhite),
                        LEDSection.SHOOTER_TOP_RIGHT),
                fire2012Palette(0.5, 0.25,
                        List.of(Color.kBlack, Color.kRed, Color.kOrangeRed, Color.kOrange, Color.kWhite),
                        LEDSection.SHOOTER_TOP_LEFT)),
        FIRE(
                fire2012Palette(0.8, 0.4, List.of(Color.kBlack, Color.kRed, Color.kOrange, Color.kWhite),
                        LEDSection.ELEVATOR_LEFT),
                fire2012Palette(0.8, 0.4, List.of(Color.kBlack, Color.kRed, Color.kOrange, Color.kWhite),
                        LEDSection.ELEVATOR_RIGHT)),
        NORMAL_FIRE(fire2012(0.8, 0.8, LEDSection.SHOOTER_RIGHT));

        private List<Consumer<AddressableLEDBuffer>> bufferConsumers;

        @SafeVarargs
        private LEDState(Consumer<AddressableLEDBuffer>... bufferConsumer) {
            this.bufferConsumers = Arrays.asList(bufferConsumer);
        }
    }

    /**
     * Initializes the LEDs.
     */
    public LEDs() {
        leds.setLength(LENGTH);
        leds.setData(buffer);
        leds.start();

        loadingNotifier = new Notifier(
                () -> {
                    synchronized (this) {
                        breathe(Color.kWhite, 3, 0, 255, LEDSection.ALL).accept(buffer);
                        leds.setData(buffer);
                    }
                });
        loadingNotifier.startPeriodic(0.02);

        setDefaultCommand(updateBufferCommand());
    }

    /**
     * Creates a command that requests a specific LED state to be active. When
     * command is cancelled, the state will no longer be active.
     * 
     * @param state the state to request
     * @return the command
     */
    public Command requestStateCommand(LEDState state) {
        return Commands.run(() -> currentStates.add(state)).ignoringDisable(true);
    }

    /**
     * Creates a command that updates the LED buffer with the contents of the
     * current LED states.
     * 
     * @return
     */
    public Command updateBufferCommand() {
        return run(() -> {
            loadingNotifier.stop();
            clear();
            // default LED states
            currentStates.addAll(DEFAULT_STATES);
            currentStates.sort((s1, s2) -> s2.ordinal() - s1.ordinal());
            currentStates.forEach(s -> s.bufferConsumers.forEach(c -> c.accept(buffer)));
            leds.setData(buffer);
            currentStates.clear();
        })
                .ignoringDisable(true)
                .withName("leds.updateBuffer");
    }

    /** Clears the buffer. */
    public void clear() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, Color.kBlack);
        }
    }
}
