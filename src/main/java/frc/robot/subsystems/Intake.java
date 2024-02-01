package frc.robot.subsystems;
import java.util.function.Function;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.techhounds.houndutil.houndlib.SparkConfigurator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

public class Intake {
    private DigitalInput beamBreak;
    //Function<CANSparkBase, REVLibError> frontMotor2 = frontMotor2::setSmartCurrentLimit;
    public Intake() {
        SparkConfigurator.createSparkFlex(0, MotorType.kBrushless, false, (s) -> s.setIdleMode(IdleMode.kBrake, (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT), (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_RADIANS),(s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_RADIANS / 60.0)));
        SparkConfigurator.createSparkFlex(1, MotorType.kBrushless, false, Function<>);
        SparkConfigurator.createSparkFlex(2, MotorType.kBrushless, false, Function<>);
        beamBreak = new DigitalInput(0);
    }
    private CANSparkFlex frontMotor = new CANSparkFlex(0, MotorType.kBrushless);
    private CANSparkFlex leftMotor = new CANSparkFlex(1, MotorType.kBrushless);
    private CANSparkFlex rightMotor = new CANSparkFlex(2, MotorType.kBrushless);
    private double velocity;

    public Command startIntake(){
        
    }

    public Command stopIntake(){

    }

    public Command liftIntake(double angle) {

    }

    public Command intakeHold(DigitalInput beamBreak) {

    }

    public Command reverseMotors() {

    }

    public double getIntakePosition(){

    }

    public double getVelocity(CANSparkFlex motor) {
        velocity = motor.getEncoder().getVelocity();
        return(velocity);
    }

    public void resetPose() {
        
    }

    public void setVoltage(double voltage, CANSparkFlex motor) {
        motor.setVoltage(voltage);
    }

}

