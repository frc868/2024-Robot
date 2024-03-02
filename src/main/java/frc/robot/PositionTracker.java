package frc.robot;

import java.util.function.Supplier;

public class PositionTracker {
    private Supplier<Double> intakePositionSupplier;
    private Supplier<Double> shooterTiltAngleSupplier;
    private Supplier<Double> climberPositionSupplier;
    private Supplier<Double> noteLiftPositionSupplier;

    public void setIntakePositionSupplier(Supplier<Double> intakePositionSupplier) {
        this.intakePositionSupplier = intakePositionSupplier;
    }

    public void setShooterTiltAngleSupplier(Supplier<Double> shooterTiltAngleSupplier) {
        this.shooterTiltAngleSupplier = shooterTiltAngleSupplier;
    }

    public void setClimberPositionSupplier(Supplier<Double> climberPositionSupplier) {
        this.climberPositionSupplier = climberPositionSupplier;
    }

    public void setNoteLiftPositionSupplier(Supplier<Double> noteLiftPositionSupplier) {
        this.noteLiftPositionSupplier = noteLiftPositionSupplier;
    }

    public double getIntakePosition() {
        return intakePositionSupplier.get();
    }

    public double getShooterTiltAngle() {
        return shooterTiltAngleSupplier.get();
    }

    public double getClimberPosition() {
        return climberPositionSupplier.get();
    }

    public double getNoteLiftPosition() {
        return noteLiftPositionSupplier.get();
    }
}
