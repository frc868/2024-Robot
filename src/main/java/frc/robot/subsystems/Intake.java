package frc.robot.subsystems;

import java.util.function.Function;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.techhounds.houndutil.houndlib.SparkConfigurator;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Intake.*;

/*
 * The Intake subsystem, which pulls in a note and can be sent to a shooter or can be used to score the amp. 
 * @author rb, jq, ak
 */
public class Intake extends SubsystemBase {
    private DigitalInput beamBreak;
    private CANSparkFlex leftMotor, rightMotor, frontMotor;
    private ProfiledPIDController pidController;
    private ArmFeedforward intakeFeedForward;

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
        pidController = new ProfiledPIDController(kP, kI, kD,
                new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        intakeFeedForward = new ArmFeedforward(kS, kG, kV, kA);

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

    public Command runFrontMotorCommand() {
        return startEnd(
                () -> frontMotor.setVoltage(6),
                () -> frontMotor.setVoltage(0))
                .withName("Run Front Motors");
    }

    public Command startFrontMotorCommand() {
        return runOnce(() -> frontMotor.setVoltage(6))
                .withName("Start Front Motors");
    }

    public Command stopFrontMotorCommand() {
        return runOnce(() -> frontMotor.setVoltage(0))
                .withName("Stop Front Motors");
    }

    public Command reverseFrontMotorCommand() {
        return startEnd(
                () -> frontMotor.setVoltage(-6),
                () -> frontMotor.setVoltage(0))
                .withName("Run Front Motors");
    }

    public Command moveToCurrentGoalCommand() {
        double feedback = pidController.calculate(getIntakePosition());
        double feedforward = intakeFeedForward.calculate(pidController.getSetpoint().position,
                pidController.getSetpoint().velocity);
    }

    public Command liftIntake(double angle) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getIntakePosition())),
                runOnce(() -> pidController.setGoal(angle)),
                moveToCurrentGoalCommand().until(pidController::atGoal)).withTimeout(2)
                .finallyDo((d) -> .stopLiftingMotors())
                .withName("Move to Position");
    

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

    public Command moveToPosition() {
        return Commands.sequence(
            runOnce(pidController = new ProfiledPIDController(CURRENT_LIMIT, BEAM_BREAK_ID, BEAM_BREAK_CHANNEL, null));
        );
    }

    public double getIntakePosition() {
        return leftMotor.getEncoder().getPosition();
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

    public void stopLiftingMotors() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

}