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

    /*
     * Starts running the motor that is inputted in the parameter.
     * 
     * @param CANSparkFlex motor
     * 
     * @return the command
     */
    public Command startIntake(CANSparkFlex motor) {
        return runOnce(() -> {
            motor.setVoltage(1);
        });
    }

    /*
     * Stops the motor that is inputted in the parameter.
     * 
     * @param CANSparkFlex motor
     * 
     * @return the command
     */
    public Command stopIntake(CANSparkFlex motor) {
        return runOnce(() -> {
            motor.setVoltage(0);
        });
    }

    /*
     * Creates a startEndCommand (requiring this subsystem) to run the passover
     * motor.
     * This will run the motors until the command is interrupted/cancelled.
     * 
     * @return the command
     */
    public Command runFrontMotorCommand() {
        return startEnd(
                () -> frontMotor.setVoltage(6),
                () -> frontMotor.setVoltage(0))
                .withName("Run Front Motors");
    }

    /*
     * ?
     */
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
        return run(() -> {
            double feedback = pidController.calculate(getIntakePosition());
            double feedforward = intakeFeedForward.calculate(pidController.getSetpoint().position,
                    pidController.getSetpoint().velocity);
            setSideVoltage(feedback + feedforward);
        }).withName("Move to curerent goal");
    }

    public Command liftIntake(double angle) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getIntakePosition())),
                runOnce(() -> pidController.setGoal(angle)),
                moveToCurrentGoalCommand().until(pidController::atGoal)).withTimeout(2)
                .finallyDo((d) -> stopLiftingMotors())
                .withName("Move to Position");

    }

    public Command intakeHold(DigitalInput beamBreak) {

    }

    public Command intakeGround() {
        // set angle later
        double targetAngle = 0;
        // runs runOnce() in order
        return Commands.sequence(
                runOnce(() -> pidController.reset(getIntakePosition())),
                runOnce(() -> pidController.setGoal(targetAngle)),
                moveToCurrentGoalCommand().until(pidController::atGoal))
                .withTimeout(2)
                .finallyDo((d) -> stopLiftingMotors())
                .withName("Move intake ground");

    }

    public Command intakeAmp() {
        // set angle later
        double targetAngleAmp = 0;
        // runs runOnce() in order
        return Commands.sequence(
                runOnce(() -> pidController.reset(getIntakePosition())),
                runOnce(() -> pidController.setGoal(targetAngleAmp)),
                moveToCurrentGoalCommand().until(pidController::atGoal))
                .withTimeout(2)
                .finallyDo((d) -> stopLiftingMotors())
                .withName("Move intake ground");

    }

    public Command intakeVertical() {
        // set angle later
        double targetAngleVertical = 0;
        // runs runOnce() in order
        return Commands.sequence(
                runOnce(() -> pidController.reset(getIntakePosition())),
                runOnce(() -> pidController.setGoal(targetAngleVertical)),
                moveToCurrentGoalCommand().until(pidController::atGoal))
                .withTimeout(2)
                .finallyDo((d) -> stopLiftingMotors())
                .withName("Move intake ground");

    }

    /*
     * public Command moveToPosition() {
     * return Commands.sequence(
     * runOnce(pidController = new ProfiledPIDController(kP, kI, kD, new
     * TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)));
     * );
     * }
     */

    /*
     * Gets the current position from the leftMotor
     * 
     * @return the current position of the motor as a double
     */
    public double getIntakePosition() {
        return leftMotor.getEncoder().getPosition();
    }

    /*
     * Gets the current velocity of the motor inputted as the parameter
     * 
     * @return the current velocity of the motor as a double
     */
    public double getVelocity(CANSparkFlex motor) {
        velocity = motor.getEncoder().getVelocity();
        return (velocity);
    }

    /*
     * Gets the current velocity of the motor inputted as the parameter
     * 
     * @return the current velocity of the motor as a double
     */
    public void resetPose(CANSparkFlex motor1, CANSparkFlex motor2) {
        motor1.getEncoder().setPosition(RESET_POSITION);
        motor2.getEncoder().setPosition(RESET_POSITION);
    }

    public void setFrontVoltage(double voltage) {
        frontMotor.setVoltage(voltage);
    }

    public void setSideVoltage(double voltage) {
        rightMotor.setVoltage(voltage);
        leftMotor.setVoltage(voltage);
    }

    public void stopLiftingMotors() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

}