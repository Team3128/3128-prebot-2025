package frc.team3128.subsystems.Arm;

public enum ArmStates {

    START(180, 0),
    NEUTRAL(-9.5, -0.05),
    HELD_NEUTRAL(170, 0),
    HANDOFF(-9.5, -1), // HANDOFF(0, -1),
    GROUND_INTAKE(90, 0),
    PRE_L1(110, 0),
    L1(90, 0),
    PRE_L2(144, 0),
    L2(109, 0),
    PRE_L3(140, 0),
    L3(95, 0),
    PRE_L4(150, 0),
    L4(80, 0),
    PRE_L3_BACK(220, 0),
    L3_BACK(270, 0),
    PRE_L4_BACK(210, 0),
    L4_BACK(280, 0),
    ALGAE_1(90, 0.8),
    ALGAE_2(90, 0.8),
    ALGAE_BARGE(135, 0.5),
    CLIMB(125, 0);

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
