package frc.team3128.subsystems.Arm;

public enum ArmStates {

    NEUTRAL(0, -0.05),
    HANDOFF(0, -0.5),
    PRE_L1(-250, -0.05),
    L1(-270, 0.5),
    PRE_L2(-250, -0.05),
    L2(-270, 0.5),
    PRE_L3(-250, -0.05),
    L3(-270, 0.5),
    PRE_L4(-250, -0.05),
    L4(-270, 0.5),
    ALGAE_1(-270, -0.5),
    ALGAE_2(-270, -0.5),
    ALGAE_BARGE(-235, 0.5);

    private final double angle;
    private final double power;

    ArmStates(double angle, double power) {
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
