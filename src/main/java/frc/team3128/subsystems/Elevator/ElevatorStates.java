package frc.team3128.subsystems.Elevator;

public enum ElevatorStates {

    NEUTRAL(0.5),
    L1(0.5),
    L2(0.7),
    L3(0.9),
    L4(1.3),
    ALGAE_1(0.6),
    ALGAE_2(0.8),
    ALGAE_BARGE(1.5),
    GROUND_INTAKE(0);

    private final double setpoint;

    ElevatorStates(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

}
