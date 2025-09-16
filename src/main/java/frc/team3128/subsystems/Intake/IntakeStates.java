package frc.team3128.subsystems.Intake;

public enum IntakeStates {

    NEUTRAL(0, -.05),
    INTAKE(125, -1),
    OUTTAKE(20, 0.7),
    HANDOFF(0, 0.5);

    private final double angle;
    private final double power;

    IntakeStates(double angle, double power) {
        this.angle = angle;
        this.power = power;
    }

    public double getAngle() {
        return angle;
    }

    public double getPower() {
        return power;
    }
    
}
