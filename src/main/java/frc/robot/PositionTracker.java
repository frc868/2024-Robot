package frc.robot;

import java.util.function.Supplier;

public class PositionTracker {
    private Supplier<Double> intakePositionSupplier;
    private Supplier<Double> shooterTiltPositionSupplier;
    private Supplier<Double> climberPositionSupplier;
    private Supplier<Double> noteLiftPositionSupplier;

    public void setIntakePositionSupplier(Supplier<Double> intakePositionSupplier) {
        this.intakePositionSupplier = intakePositionSupplier;
    }

    public void setShooterTiltPositionSupplier(Supplier<Double> shooterTiltPositionSupplier) {
        this.shooterTiltPositionSupplier = shooterTiltPositionSupplier;
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

    public double getShooterTiltPosition() {
        return shooterTiltPositionSupplier.get();
    }

    public double getClimberPosition() {
        return climberPositionSupplier.get();
    }

    public double getNoteLiftPosition() {
        return noteLiftPositionSupplier.get();
    }
}
