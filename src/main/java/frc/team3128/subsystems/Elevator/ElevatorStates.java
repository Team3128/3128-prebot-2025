package frc.team3128.subsystems.Elevator;

public enum ElevatorStates {

    START(0),
    // NEUTRAL(0.95),
    NEUTRAL(0.94),
    HELD_NEUTRAL(0),
    L1(0.22),
    L2(0.22+.05),
    L3(0.69+.05),
    L4(1.29+.05),
    CORAL_LOLLIPOP(0.1),
    ALGAE_LOLLIPOP(0.5),
    ALGAE_GROUND(0),
    ALGAE_1(0.67),
    ALGAE_2(1.21),
    ALGAE_BARGE(1.5);

    private final double setpoint;

    ElevatorStates(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

}
