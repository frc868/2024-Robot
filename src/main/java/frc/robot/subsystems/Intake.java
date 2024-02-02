package frc.robot.subsystems;

import java.util.function.Function;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.techhounds.houndutil.houndlib.SparkConfigurator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Intake.*;

/*
 * The Intake subsystem, which pulls in a note and can be sent to a shooter or can be used to score the amp. 
 * @author rb, jq, ak
 */
public class Intake extends SubsystemBase {
    private DigitalInput beamBreak;
    private CANSparkFlex leftMotor, rightMotor, frontMotor;

    // Function<CANSparkBase, REVLibError> frontMotor2 =
    // frontMotor2::setSmartCurrentLimit;
    public Intake() {
        leftMotor = SparkConfigurator.createSparkFlex(INTAKE_MOTOR_ID, MotorType.kBrushless, false,
                (s) -> s.setIdleMode(IdleMode.kBrake), (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_RADIANS),
                (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_RADIANS / 60.0));
        rightMotor = SparkConfigurator.createSparkFlex(LIFTING_MOTOR_1_ID, MotorType.kBrushless, false,
                (s) -> s.setIdleMode(IdleMode.kBrake), (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_RADIANS),
                (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_RADIANS / 60.0));
        frontMotor = SparkConfigurator.createSparkFlex(LIFTING_MOTOR_2_ID, MotorType.kBrushless, false,
                (s) -> s.setIdleMode(IdleMode.kBrake), (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_RADIANS),
                (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_RADIANS / 60.0));
        SparkConfigurator.safeBurnFlash();
        beamBreak = new DigitalInput(BEAM_BREAK_CHANNEL);
    }

    private double velocity;

    public Command startIntake(CANSparkFlex motor) {
        return runOnce(() -> {
            motor.setVoltage(1);
        });

    }

    public Command stopIntake(CANSparkFlex motor) {
        return runOnce(() -> {
            motor.setVoltage(0);
        });

    }

    public Command liftIntake(double angle) {

    }

    public Command intakeHold(DigitalInput beamBreak) {

    }

    public Command reverseMotors() {

    }

    public Command intakeGround() {

    }

    public Command intakeAmp() {

    }

    public Command intakeVertical() {
        resetPose(leftMotor, rightMotor);
    }

    public double getIntakePosition(CANSparkFlex motor) {
        return motor.getEncoder().getPosition();
    }

    public double getVelocity(CANSparkFlex motor) {
        velocity = motor.getEncoder().getVelocity();
        return (velocity);
    }

    public void resetPose(CANSparkFlex motor1, CANSparkFlex motor2) {
        motor1.getEncoder().setPosition(RESET_POSITION);
        motor2.getEncoder().setPosition(RESET_POSITION);
    }

    public void setVoltage(double voltage, CANSparkFlex motor) {
        motor.setVoltage(voltage);
    }

}
