package frc.team3128.subsystems.Elevator;

public enum ElevatorStates {

    START(0),
    NEUTRAL(0.95),
    HELD_NEUTRAL(0),
    L1(0.5),
    L2(0.7),
    L3(0.69),
    L4(1.3),
    CORAL_LOLLIPOP(0.1),
    ALGAE_LOLLIPOP(0.5),
    ALGAE_GROUND(0),
    ALGAE_1(0.6),
    ALGAE_2(0.8),
    ALGAE_BARGE(1.5);

    private final double setpoint;

    ElevatorStates(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

}
