package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LEDs.*;
import static com.techhounds.houndutil.houndlib.leds.LEDPatterns.*;

/**
 * The LEDs subsystem, which controls the state of the LEDs through an internal
 * state machine and continuously updates the LED's buffer.
 * 
 * @author dr
 */
@LoggedObject
public class LEDs extends SubsystemBase {
    /** The LEDs. */
    private AddressableLED leds = new AddressableLED(9);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(LENGTH);

    private ArrayList<LEDState> currentStates = new ArrayList<LEDState>();

    public enum LEDState {
        OFF(solid(Color.kBlack, LEDSection.ALL)),
        RED_BREATHE(breathe(Color.kRed, 3, 0, 255, LEDSection.ALL)),
        NOTE_LIFT_UNINITIALIZED(
                breathe(Color.kRed, 3, 0, 255, LEDSection.ELEVATOR_LEFT_TOP),
                breathe(Color.kRed, 3, 0, 255, LEDSection.ELEVATOR_RIGHT_TOP)),
        CLIMBER_UNINITIALIZED(
                breathe(Color.kRed, 3, 0, 255, LEDSection.ELEVATOR_LEFT_BOTTOM),
                breathe(Color.kRed, 3, 0, 255, LEDSection.ELEVATOR_RIGHT_BOTTOM)),
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
        SUBWOOFER_ONLY(flash(Color.kYellow, 1, LEDSection.SHOOTER_TOP)),
        PODIUM_ONLY(flash(Color.kBlue, 1, LEDSection.SHOOTER_TOP)),
        SOLID_BLUE(solid(Color.kBlue, LEDSection.ALL)),
        SOLID_GREEN(solid(Color.kGreen, LEDSection.ALL)),
        FLASHING_AQUA(flash(Color.kAqua, 0.5, LEDSection.ALL)),
        PURPLE_WAVE(wave(new Color("#9000DD"), 30, 20, 100, 255,
                LEDSection.SHOOTER)),
        RAINBOW(rainbow(255, 3, LEDSection.ALL)),
        PURPLE_FIRE(
                fire2012Palette(0.8, 0.4,
                        List.of(Color.kBlack, new Color("#ad3fe8"), new Color("#9000DD"), new Color("#400063")),
                        LEDSection.ELEVATOR_LEFT),
                fire2012Palette(0.8, 0.4,
                        List.of(Color.kBlack, new Color("#ad3fe8"), new Color("#9000DD"), new Color("#400063")),
                        LEDSection.ELEVATOR_RIGHT)),
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

        setDefaultCommand(updateStateMachineCommand());
    }

    public Command requestStateCommand(LEDState state) {
        return Commands.run(() -> currentStates.add(state)).ignoringDisable(true);
    }

    public Command updateStateMachineCommand() {
        return run(() -> {
            clear();
            currentStates.add(LEDState.PURPLE_WAVE);
            currentStates.add(LEDState.PURPLE_FIRE);
            currentStates.sort((s1, s2) -> s2.ordinal() - s1.ordinal());
            currentStates.forEach(s -> s.bufferConsumers.forEach(c -> c.accept(buffer)));
            leds.setData(buffer);
            currentStates.clear();
        })
                .ignoringDisable(true)
                .withName("leds.updateStateMachine");
    }

    public void clear() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, Color.kBlack);
        }
    }
}
