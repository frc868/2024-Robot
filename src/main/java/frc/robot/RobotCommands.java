package frc.robot;

import static frc.robot.Constants.Shooter.SHOOTING_RPS;

import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive.DriveMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTilt;

public class RobotCommands {
    public static Command targetSpeakerCommand(Drivetrain drivetrain, Shooter shooter, ShooterTilt shooterTilt) {
        return Commands.parallel(
                drivetrain.controlledRotateCommand(() -> {
                    Transform2d diff = drivetrain.getPose().minus(FieldConstants.TARGET.toPose2d());
                    Rotation2d rot = new Rotation2d(diff.getX(), diff.getY());
                    if (DriverStation.getAlliance().isPresent() &&
                            DriverStation.getAlliance().get() == Alliance.Blue)
                        rot = rot.plus(new Rotation2d(Math.PI));
                    return rot.getRadians();
                }, DriveMode.FIELD_ORIENTED),
                shooterTilt.targetSpeakerCommand(drivetrain::getPose).asProxy(),
                shooter.spinAtVelocityCommand(() -> SHOOTING_RPS).asProxy());
    }

    public static Command shootCommand(Drivetrain drivetrain, Intake intake, Shooter shooter, ShooterTilt shooterTilt) {
        return Commands.sequence(
                shooter.spinAtVelocityCommand(() -> SHOOTING_RPS).asProxy()
                        .until(() -> shooterTilt.atGoal() && shooter.atGoal()),
                shooter.spinAtVelocityCommand(() -> SHOOTING_RPS).asProxy().alongWith(intake.runRollersCommand())
                        .withTimeout(0.2));
    }
}
