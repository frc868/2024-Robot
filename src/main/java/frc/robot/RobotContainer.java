package frc.robot;

import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.SparkConfigurator;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.SendableLog;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.NoteLift;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTilt;
import frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.function.Supplier;

/**
 * The container for the robot. Contains subsystems, OI devices, and commands.
 */
public class RobotContainer {
    @SendableLog(groups = "wpilib")
    public static Mechanism2d mechanisms = new Mechanism2d(5, 2);

    @Log(groups = "subsystems")
    @SendableLog(groups = { "wpilib", "subsystems" })
    private final Drivetrain drivetrain = new Drivetrain();

    @Log(groups = "subsystems")
    @SendableLog(groups = { "wpilib", "subsystems" })
    private final Intake intake = new Intake();

    @Log(groups = "subsystems")
    @SendableLog(groups = { "wpilib", "subsystems" })
    private final Shooter shooter = new Shooter();

    @Log(groups = "subsystems")
    @SendableLog(groups = { "wpilib", "subsystems" })
    private final ShooterTilt shooterTilt = new ShooterTilt();

    @Log(groups = "subsystems")
    @SendableLog(groups = { "wpilib", "subsystems" })
    private final Climber climber = new Climber();

    @Log(groups = "subsystems")
    @SendableLog(groups = { "wpilib", "subsystems" })
    private final NoteLift noteLift = new NoteLift();

    @Log(groups = "subsystems")
    @SendableLog(groups = { "wpilib", "subsystems" })
    private final Vision vision = new Vision();

    @Log(groups = "subsystems")
    @SendableLog(groups = { "wpilib", "subsystems" })
    private final LEDs leds = new LEDs();

    @Log(groups = { "subsystems", "misc" })
    private final PowerDistribution pdh = new PowerDistribution();

    @Log(groups = { "subsystems", "misc" })
    private final PneumaticHub ph = new PneumaticHub();

    @SendableLog(groups = "wpilib")
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();

    private double prevLoopTime = 0.0;

    @Log(groups = "wpilib")
    private final Supplier<Double> loopTimeMs = () -> {
        double timestamp = Timer.getFPGATimestamp();
        double loopTime = Timer.getFPGATimestamp() - prevLoopTime;
        prevLoopTime = timestamp;
        return loopTime * 1000.0;
    };

    /**
     * Constructs the robot container.
     */
    public RobotContainer() {
        vision.setPoseEstimator(drivetrain.getPoseEstimator());
        vision.setSimPoseSupplier(drivetrain::getSimPose);
        SparkConfigurator.safeBurnFlash();
        DataLogManager.logNetworkTables(true);
        DriverStation.startDataLog(DataLogManager.getLog());
        DataLogManager.start();

        LoggingManager.getInstance().registerRobotContainer(this);
        LoggingManager.getInstance().registerClass(LoggingManager.class, "houndlog", new ArrayList<>());
        LoggingManager.getInstance().registerClass(FieldConstants.class, "fieldConstants", new ArrayList<>());

        LiveWindow.disableAllTelemetry(); // livewindow is basically deprecated. using houndlog instead.

        if (RobotBase.isSimulation()) {
            // prevents annoying joystick disconnected warning
            DriverStation.silenceJoystickConnectionWarning(true);
        }
        configureButtonBindings();
        configureAuto();
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setEnabled(true);
    }

    private void configureButtonBindings() {
        Controls.configureDriverControl(0, drivetrain, intake, shooter, shooterTilt, climber, noteLift, leds);
    }

    private void configureAuto() {
        AutoManager.getInstance().addRoutine(Autos.autoA123(drivetrain, intake, shooter, shooterTilt));
        AutoManager.getInstance().addRoutine(Autos.autoBA123(drivetrain, intake, shooter, shooterTilt));
        AutoManager.getInstance().addRoutine(Autos.autoCBA123(drivetrain, intake, shooter, shooterTilt));
        AutoManager.getInstance().addRoutine(Autos.auto453(drivetrain, intake, shooter, shooterTilt));
        AutoManager.getInstance().addRoutine(Autos.autoBAC(drivetrain, intake, shooter, shooterTilt));
        AutoManager.getInstance().addRoutine(Autos.auto1234(drivetrain, intake, shooter, shooterTilt));
    }

    @Log
    private Pose3d[] getComponentPoses() {
        return new Pose3d[] {
                intake.getComponentPose(),
                shooterTilt.getShooterComponentPose(),
                shooterTilt.getOuterLeadScrewComponentPose(),
                shooterTilt.getInnerLeadScrewComponentPose(),
                climber.getComponentPose(),
                noteLift.getComponentPose()
        };
    }
}