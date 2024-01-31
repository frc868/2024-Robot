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

    PIDController PID = new PIDController(kP, kI, kD);

    @Override
    public double getVelocity() { //in RPM
        velocity = yay.getEncoder().getVelocity();
        return(velocity);
    }

    @Override
    public void setVoltage(double voltage) { //2
        yay.setVoltage(voltage);
    }

    @Override
    public Command spinAtVelocityCommand(Supplier<Double> goalVelocitySupplier) { //1
        return run(() ->
            double calculatedSpeed = PID.calculate(getVelocity(), goalVelocitySupplier.get()); //change 1 to desired RPM to shoot
            yay.setVoltage(12 * calculatedSpeed);
        );
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {

    }

    @Override
    public Command coastMotorsCommand() {

    }
}