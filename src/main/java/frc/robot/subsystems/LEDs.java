package frc.robot.subsystems;

import java.util.ArrayList;
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
    private AddressableLED leds = new AddressableLED(PORT);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(LENGTH);

    private ArrayList<LEDState> currentStates = new ArrayList<LEDState>();

    public enum LEDState {
        OFF(solid(Color.kWhite, LEDSection.SHOOTER_RIGHT)),
        CHASE(chase(Color.kPurple, Color.kPurple, 20, 0.5, 100, false, LEDSection.SHOOTER_RIGHT)),
        BREATHE(breathe(Color.kWhite, 3, 0, 255, LEDSection.SHOOTER_RIGHT)),
        WAVE(wave(Color.kPurple, 20, 20, 150, 255, LEDSection.SHOOTER_RIGHT)),
        FIRE(fire(0.3, 0.02, 0.75, LEDSection.SHOOTER_RIGHT, Color.kRed, Color.kOrange, Color.kYellow, 50)),
        FLASHING_RED(flash(Color.kRed, 0.1, LEDSection.SHOOTER_RIGHT)),
        FLASHING_ORANGE(flash(Color.kOrange, 1, LEDSection.SHOOTER_RIGHT));

        private Consumer<AddressableLEDBuffer> bufferConsumer;

        private LEDState(Consumer<AddressableLEDBuffer> bufferConsumer) {
            this.bufferConsumer = bufferConsumer;
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

    public Command requestFlashingRedCommand() {
        return Commands.run(() -> currentStates.add(LEDState.FLASHING_RED)).ignoringDisable(true);
    }

    public Command requestChaseCommand() {
        return Commands.run(() -> currentStates.add(LEDState.CHASE)).withTimeout(0.5).ignoringDisable(true);
    }

    public Command requestFireCommand() {
        return Commands.run(() -> currentStates.add(LEDState.FIRE)).ignoringDisable(true);
    }

    public Command requestWaveCommand() {
        return Commands.run(() -> currentStates.add(LEDState.WAVE)).ignoringDisable(true);
    }

    public Command requestBreatheCommand() {
        return Commands.run(() -> currentStates.add(LEDState.BREATHE)).ignoringDisable(true);
    }

    public Command updateStateMachineCommand() {
        return run(() -> {
            clear();
            currentStates.add(LEDState.OFF);
            currentStates.sort((s1, s2) -> s2.ordinal() - s1.ordinal());
            currentStates.forEach((s) -> s.bufferConsumer.accept(buffer));
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
