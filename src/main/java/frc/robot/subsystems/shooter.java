package frc.robot.subsystems;

import java.util.function.Supplier;
import com.techhounds.houndutil.houndlib.subsystems.BaseShooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooter extends SubsystemBase implements BaseShooter {
    @Override
    public double getVelocity() {
        return(1.23); //getting rid of error
    }

    @Override
    public void setVoltage(double voltage) {

    }

    @Override
    public Command spinAtVelocityCommand(Supplier<Double> goalVelocitySupplier) {

    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {

    }

    @Override
    public Command coastMotorsCommand() {

    }
}
