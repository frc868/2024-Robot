package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkFlex;
import com.techhounds.houndutil.houndlib.subsystems.BaseShooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.controller.PIDController;

public class shooter extends SubsystemBase implements BaseShooter {

    //example motor
    public CANSparkFlex yay = new CANSparkFlex(0, CANSparkLowLevel.MotorType.kBrushless);
    //example velocity variable
    public double velocity;

    @Override
    public double getVelocity() {
        yay.getEncoder();
        velocity = 1.23; //somehow find the speed (probably something with the encoder) ¯\_(ツ)_/¯
        return(velocity);
    }

    @Override
    public void setVoltage(double voltage) {
        yay.setVoltage(voltage);
    }

    @Override
    public Command spinAtVelocityCommand(Supplier<Double> goalVelocitySupplier) {
        //I still think it's going to be more complex than this
        return runOnce(() -> yay.setVoltage(12)); //gets voltage from PD or something
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {

    }

    @Override
    public Command coastMotorsCommand() {

    }
}