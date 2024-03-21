package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndauto.Reflector;
import com.techhounds.houndutil.houndlib.SparkConfigurator;
import com.techhounds.houndutil.houndlog.LogGroup;
import com.techhounds.houndutil.houndlog.LogProfileBuilder;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.SendableLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.NoteLift;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTilt;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.LEDs.LEDState;
import frc.robot.utils.TrajectoryCalcs;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.littletonrobotics.urcl.URCL;

/**
 * The container for the robot. Contains subsystems, OI devices, and commands.
 */
public class RobotContainer {
    public final PositionTracker positionTracker = new PositionTracker();

    @Log(groups = "subsystems")
    private final Drivetrain drivetrain = new Drivetrain();

    @Log(groups = "subsystems")
    private final Intake intake = new Intake(positionTracker);

    @Log(groups = "subsystems")
    private final Shooter shooter = new Shooter();

    @Log(groups = "subsystems")
    private final ShooterTilt shooterTilt = new ShooterTilt(positionTracker);

    @Log(groups = "subsystems")
    private final Climber climber = new Climber(positionTracker);

    @Log(groups = "subsystems")
    private final NoteLift noteLift = new NoteLift(positionTracker);

    @Log(groups = "subsystems")
    private final Vision vision = new Vision();

    @Log(groups = "subsystems")
    private final LEDs leds = new LEDs();

    @Log(groups = "subsystems")
    private final HoundBrian houndBrian = new HoundBrian(drivetrain, intake, shooterTilt, climber, noteLift, leds);

    @Log(groups = { "subsystems", "misc" })
    private final PowerDistribution pdh = new PowerDistribution();

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

    @Log(groups = "wpilib")
    private final Supplier<Double> matchTimer = DriverStation::getMatchTime;

    @Log
    private final Supplier<Double> dist = () -> {
        Pose3d target = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red
                        ? Reflector.reflectPose3d(FieldConstants.SPEAKER_TARGET,
                                FieldConstants.FIELD_LENGTH)
                        : FieldConstants.SPEAKER_TARGET;

        Transform3d diff = new Pose3d(drivetrain.getPose()).minus(target);
        return new Translation2d(diff.getX(), diff.getY()).getNorm();
    };

    @Log
    private final Supplier<Pose3d> shootOnTheMovePose = () -> {
        return TrajectoryCalcs.calculateEffectiveTargetLocation(drivetrain.getPose(),
                drivetrain.getFieldRelativeSpeeds(), drivetrain.getFieldRelativeAccelerations());
    };
    @Log
    private final Supplier<Double> shotTime = () -> {
        return TrajectoryCalcs.getTimeToShoot(drivetrain.getPose(),
                TrajectoryCalcs.calculateEffectiveTargetLocation(drivetrain.getPose(),
                        drivetrain.getFieldRelativeSpeeds(), drivetrain.getFieldRelativeAccelerations()));
    };

    @Log
    private final Supplier<Boolean> initialized = GlobalStates.INITIALIZED::enabled;

    /**
     * Constructs the robot container.
     */
    public RobotContainer() {
        vision.setPoseEstimator(drivetrain.getPoseEstimator());
        vision.setVisionMeasurementConsumer(drivetrain::addVisionMeasurement);
        vision.setSimPoseSupplier(drivetrain::getSimPose);
        vision.setSpeedsSupplier(drivetrain::getChassisSpeeds);

        positionTracker.setIntakePositionSupplier(intake::getPosition);
        positionTracker.setShooterTiltAngleSupplier(shooterTilt::getAngle);
        positionTracker.setClimberPositionSupplier(climber::getPosition);
        positionTracker.setNoteLiftPositionSupplier(noteLift::getPosition);

        SparkConfigurator.safeBurnFlash();
        DataLogManager.logNetworkTables(true);
        DriverStation.startDataLog(DataLogManager.getLog());
        DataLogManager.start();
        URCL.start();
        SignalLogger.start();

        LoggingManager.getInstance().registerRobotContainer(this);
        LoggingManager.getInstance().registerClass(LoggingManager.class, "houndlog", new ArrayList<>());
        LoggingManager.getInstance()
                .addGroup(new LogGroup("robotController", LogProfileBuilder.buildRobotControllerLogItems()));
        LiveWindow.disableAllTelemetry(); // livewindow is basically deprecated. using houndlog instead.

        if (RobotBase.isSimulation()) {
            // prevents annoying joystick disconnected warning
            DriverStation.silenceJoystickConnectionWarning(true);
        }
        configureButtonBindings();
        configureAuto();

        new Trigger(DriverStation::isEnabled)
                .onTrue(Commands.parallel(
                        intake.resetControllersCommand(),
                        shooterTilt.resetControllersCommand(),
                        noteLift.resetControllersCommand(),
                        climber.resetControllersCommand()).withName("resetControllers"));

        new Trigger(() -> {
            return intake.getInitialized()
                    && shooterTilt.getInitialized()
                    && climber.getInitialized()
                    && noteLift.getInitialized();
        }).onTrue(GlobalStates.INITIALIZED.enableCommand());

        intake.noteInIntakeFromOutsideTrigger.debounce(0.15)
                .onTrue(leds.requestStateCommand(LEDState.SOLID_GREEN).withTimeout(1));
        intake.noteInShooterTrigger.onTrue(leds.requestStateCommand(LEDState.SOLID_BLUE).withTimeout(1));

        leds.requestStateCommand(LEDState.INITIALIZATION_BLACK_BACKGROUND).until(GlobalStates.INITIALIZED::enabled)
                .schedule();
        leds.requestStateCommand(LEDState.INTAKE_UNINITIALIZED).until(intake::getInitialized).schedule();
        leds.requestStateCommand(LEDState.SHOOTER_TILT_UNINITIALIZED).until(shooterTilt::getInitialized).schedule();
        leds.requestStateCommand(LEDState.CLIMBER_UNINITIALIZED).until(climber::getInitialized).schedule();
        leds.requestStateCommand(LEDState.NOTE_LIFT_UNINITIALIZED).until(noteLift::getInitialized).schedule();

        new Trigger(GlobalStates.INITIALIZED::enabled)
                .onTrue(leds.requestStateCommand(LEDState.INITIALIZED_CONFIRM).withTimeout(3));
    }

    private void configureButtonBindings() {
        Controls.configureDriverControls(0, drivetrain, intake, shooter, shooterTilt, climber, noteLift, leds);
        Controls.configureOperatorControls(1, drivetrain, intake, shooter, shooterTilt, climber, noteLift);
        Controls.configureOverridesControls(2, drivetrain, intake, shooter, shooterTilt, climber, noteLift);
        NTCommands.configureNTCommands(drivetrain, intake, shooter, shooterTilt, climber, noteLift, leds);
    }

    private void configureAuto() {
        AutoManager.getInstance().addRoutine(Autos.autoTest(drivetrain, intake,
                shooter, shooterTilt));
        AutoManager.getInstance().addRoutine(Autos.auto213(drivetrain, intake,
                shooter, shooterTilt));
        AutoManager.getInstance().addRoutine(Autos.autoCBA(drivetrain, intake,
                shooter, shooterTilt));
        AutoManager.getInstance().addRoutine(Autos.auto453(drivetrain, intake,
                shooter, shooterTilt));
        AutoManager.getInstance().addRoutine(Autos.autoCBA12(drivetrain, intake,
                shooter, shooterTilt));
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