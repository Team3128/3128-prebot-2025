package frc.team3128.subsystems.Intake;

public enum IntakeStates {

    START(0, 0),
    NEUTRAL(3, -.05),
    INTAKE(133, -0.6),
    OUTTAKE(30, 1),
    HANDOFF(3, 1);

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
