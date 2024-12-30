package frc.robot;

import static frc.robot.Constants.HoundBrian.*;

import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.ShooterTilt;

@LoggedObject
public class HoundBrian {
    @Log
    private final DigitalInput drivetrainButton = new DigitalInput(BUTTON_1);
    @Log
    private final DigitalInput intakeButton = new DigitalInput(BUTTON_2);
    @Log
    private final DigitalInput shooterTiltButton = new DigitalInput(BUTTON_3);
    @Log
    private final DigitalInput climberButton = new DigitalInput(BUTTON_4);
    @Log
    private final DigitalInput noteLiftButton = new DigitalInput(BUTTON_5);
    @Log
    private final DigitalInput actionButton = new DigitalInput(BUTTON_6);
    @Log
    private final DigitalInput actionButton2 = new DigitalInput(BUTTON_7);

    private final DIOSim drivetrainButtonSim = new DIOSim(drivetrainButton);
    private final DIOSim intakeButtonSim = new DIOSim(intakeButton);
    private final DIOSim shooterTiltButtonSim = new DIOSim(shooterTiltButton);
    private final DIOSim climberButtonSim = new DIOSim(climberButton);
    private final DIOSim noteLiftButtonSim = new DIOSim(noteLiftButton);
    private final DIOSim actionButtonSim = new DIOSim(actionButton);
    private final DIOSim actionButton2Sim = new DIOSim(actionButton2);

    public HoundBrian(Drivetrain drivetrain, Intake intake, ShooterTilt shooterTilt, Climber climber, LEDs leds) {

        new Trigger(drivetrainButton::get).negate()
                // .and(new Trigger(intakeButton::get).negate().debounce(2,
                // DebounceType.kFalling))
                .and(DriverStation::isDisabled)
                .onTrue(drivetrain.resetGyroCommand().ignoringDisable(true));
        new Trigger(intakeButton::get).negate()
                // .and(new Trigger(intakeButton::get).negate().debounce(2,
                // DebounceType.kFalling))
                .and(DriverStation::isDisabled)
                .onTrue(intake.resetPositionCommand().ignoringDisable(true));
        new Trigger(shooterTiltButton::get).negate()
                // .and(new Trigger(shooterTiltButton::get).negate().debounce(2,
                // DebounceType.kFalling))
                .and(DriverStation::isDisabled)
                .onTrue(shooterTilt.resetPositionCommand().ignoringDisable(true));
        new Trigger(climberButton::get).negate()
                // .and(new Trigger(climberButton::get).negate().debounce(2,
                // DebounceType.kFalling))
                .and(DriverStation::isDisabled)
                .onTrue(climber.resetPositionCommand().ignoringDisable(true));

        // new Trigger(intakeButton::get).debounce(3, DebounceType.kBoth)
        // .and(DriverStation::isDisabled)
        // .toggleOnTrue(intake.coastMotorsCommand());
        // new Trigger(shooterTiltButton::get).debounce(3, DebounceType.kBoth)
        // .and(DriverStation::isDisabled)
        // .toggleOnTrue(shooterTilt.coastMotorsCommand());
        // new Trigger(climberButton::get).debounce(3, DebounceType.kBoth)
        // .and(DriverStation::isDisabled)
        // .toggleOnTrue(climber.coastMotorsCommand());
        // new Trigger(noteLiftButton::get).debounce(3, DebounceType.kBoth)
        // .and(DriverStation::isDisabled)
        // .toggleOnTrue(noteLift.coastMotorsCommand());

        if (RobotBase.isSimulation()) {
            drivetrainButtonSim.setValue(true);
            intakeButtonSim.setValue(true);
            shooterTiltButtonSim.setValue(true);
            climberButtonSim.setValue(true);
            noteLiftButtonSim.setValue(true);
            actionButtonSim.setValue(true);
            actionButton2Sim.setValue(true);
        }
    }

    public Command simTriggerIntakeButton() {
        return Commands.startEnd(() -> intakeButtonSim.setValue(false), () -> intakeButtonSim.setValue(true))
                .ignoringDisable(true);
    }

}
