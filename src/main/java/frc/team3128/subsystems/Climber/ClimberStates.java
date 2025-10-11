package frc.team3128.subsystems.Climber;

public enum ClimberStates {

    START(0, 0, 0),
    NEUTRAL(0, 0.1, 0),
    PRE_CLIMB_PRIME(25, 0.1, 0),
    CLIMB_PRIME(35, 0.1, 0.5),
    CLIMB(75.0, 0.25, 0);

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
