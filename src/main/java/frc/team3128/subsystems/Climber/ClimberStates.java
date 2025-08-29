package frc.team3128.subsystems.Climber;

public enum ClimberStates {

    NEUTRAL(0, 1, 0),
    PRE_CLIMB_PRIME(70, 1, 0),
    CLIMB_PRIME(-1799.785, 1, 0.5),
    CLIMB(-3442.343, 1, 0);

    private final double angle, winchPower, rollerPower;

    private ClimberStates(double angle, double winchPower, double rollerPower) {
        this.angle = angle;
        this.winchPower = winchPower;
        this.rollerPower = rollerPower;
    }

    public double getAngle() {
        return angle;
    }

    public double getWinchPower() {
        return winchPower;
    }

    public double getRollerPower() {
        return rollerPower;
    }

}