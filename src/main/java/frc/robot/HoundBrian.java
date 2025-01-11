package frc.robot;

import static frc.robot.Constants.HoundBrian.*;

import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.ShooterTilt;

/**
 * Representation for the button board that controls mechanism initialization.
 */
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

    /**
     * Creates a new HoundBrian object. Registers triggers for initializing
     * mechanisms when their respective buttons are pressed and the robot is
     * disabled.
     * 
     * @param drivetrain  the drivetrain
     * @param intake      the intake
     * @param shooterTilt the shooter tilt
     * @param climber     the climber
     * @param leds        the LEDs
     */
    public HoundBrian(Drivetrain drivetrain, Intake intake, ShooterTilt shooterTilt, Climber climber, LEDs leds) {

        new Trigger(drivetrainButton::get).negate()
                .and(DriverStation::isDisabled)
                .onTrue(drivetrain.resetGyroCommand().ignoringDisable(true));
        new Trigger(intakeButton::get).negate()
                .and(DriverStation::isDisabled)
                .onTrue(intake.resetPositionCommand().ignoringDisable(true));
        new Trigger(shooterTiltButton::get).negate()
                .and(DriverStation::isDisabled)
                .onTrue(shooterTilt.resetPositionCommand().ignoringDisable(true));
        new Trigger(climberButton::get).negate()
                .and(DriverStation::isDisabled)
                .onTrue(climber.resetPositionCommand().ignoringDisable(true));

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
}
