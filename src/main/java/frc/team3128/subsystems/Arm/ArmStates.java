package frc.team3128.subsystems.Arm;

public enum ArmStates {

    START(180, 0),
    NEUTRAL(0, -0.5),
    HELD_NEUTRAL(170, -0.5),
    HANDOFF(0, -1),
    GROUND_INTAKE(90, -0.5),
    PRE_L1(110, -0.5),
    L1(90, 0.5),
    PRE_L2(260, -0.5),
    L2(270, 0.5),
    PRE_L3(-137, -0.5),
    L3(-100, 0),
    PRE_L4(110, -0.5),
    L4(90, 0.5),
    PRE_L3_BACK(-110, -0.5),
    L3_BACK(-90, 0.5),
    PRE_L4_BACK(-110, -0.5),
    L4_BACK(-90, 0.5),
    ALGAE_1(90, -0.5),
    ALGAE_2(90, -0.5),
    ALGAE_BARGE(135, 0.5);

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
