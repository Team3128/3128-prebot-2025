package frc.team3128.subsystems.Superstructure;

import java.util.List;

import edu.wpi.first.math.Pair;
import frc.team3128.subsystems.Arm.ArmStates;
import frc.team3128.subsystems.Elevator.ElevatorStates;
import frc.team3128.subsystems.Intake.IntakeStates;

public enum SuperstructureStates {

    START(ArmStates.START, ElevatorStates.START, IntakeStates.START),
    NEUTRAL(ArmStates.NEUTRAL, ElevatorStates.NEUTRAL, IntakeStates.NEUTRAL),
    HELD_NEUTRAL(ArmStates.HELD_NEUTRAL, ElevatorStates.HELD_NEUTRAL, IntakeStates.NEUTRAL),

    CORAL_GROUND(ArmStates.NEUTRAL, ElevatorStates.NEUTRAL, IntakeStates.INTAKE),
    CORAL_GROUND_L1(ArmStates.HELD_NEUTRAL, ElevatorStates.HELD_NEUTRAL, IntakeStates.INTAKE),
    CORAL_LOLLIPOP(ArmStates.GROUND_INTAKE, ElevatorStates.CORAL_LOLLIPOP, IntakeStates.NEUTRAL),
    ALGAE_GROUND(ArmStates.GROUND_INTAKE, ElevatorStates.ALGAE_GROUND, IntakeStates.NEUTRAL),
    ALGAE_LOLLIPOP(ArmStates.GROUND_INTAKE, ElevatorStates.ALGAE_LOLLIPOP, IntakeStates.NEUTRAL),
    HANDOFF(ArmStates.HANDOFF, ElevatorStates.NEUTRAL, IntakeStates.HANDOFF),
    OUTTAKE(ArmStates.HELD_NEUTRAL, ElevatorStates.HELD_NEUTRAL, IntakeStates.OUTTAKE),

    ALGAE_1(ArmStates.ALGAE_1, ElevatorStates.ALGAE_1, IntakeStates.NEUTRAL),
    ALGAE_2(ArmStates.ALGAE_2, ElevatorStates.ALGAE_2, IntakeStates.NEUTRAL),
    ALGAE_BARGE(ArmStates.ALGAE_BARGE, ElevatorStates.ALGAE_BARGE, IntakeStates.NEUTRAL),

    PRE_L2(ArmStates.PRE_L2, ElevatorStates.L2, IntakeStates.NEUTRAL, true, false),
    PRE_L3(ArmStates.PRE_L3, ElevatorStates.L3, IntakeStates.NEUTRAL, true, false),
    PRE_L4(ArmStates.PRE_L4, ElevatorStates.L4, IntakeStates.NEUTRAL, true, false),
    PRE_L3_BACK(ArmStates.PRE_L3_BACK, ElevatorStates.L3, IntakeStates.NEUTRAL, true, false),
    PRE_L4_BACK(ArmStates.PRE_L4_BACK, ElevatorStates.L4, IntakeStates.NEUTRAL, true, false),

    L2(ArmStates.L2, ElevatorStates.L2, IntakeStates.NEUTRAL, false, true),
    L3(ArmStates.L3, ElevatorStates.L3, IntakeStates.NEUTRAL, false, true),
    L4(ArmStates.L4, ElevatorStates.L4, IntakeStates.NEUTRAL, false, true),
    L3_BACK(ArmStates.L3_BACK, ElevatorStates.L3, IntakeStates.NEUTRAL, false, true),
    L4_BACK(ArmStates.L4_BACK, ElevatorStates.L4, IntakeStates.NEUTRAL, false, true),
    
    CLIMB(ArmStates.CLIMB, ElevatorStates.HELD_NEUTRAL, IntakeStates.NEUTRAL);

    private final ArmStates arm;
    private final ElevatorStates elevator;
    private final IntakeStates intake;
    private final boolean waitClose, waitFull;

    SuperstructureStates(ArmStates arm, ElevatorStates elevator, IntakeStates intake, boolean waitClose, boolean waitFull) {
        this.arm = arm;
        this.elevator = elevator;
        this.intake = intake;
        this.waitClose = waitClose;
        this.waitFull = waitFull;
    }

    SuperstructureStates(ArmStates arm, ElevatorStates elevator, IntakeStates intake) {
        this(arm, elevator, intake, false, false);
    }

    public ArmStates getArm() {
        return arm;
    }

    public ElevatorStates getElevator() {
        return elevator;
    }

    public IntakeStates getIntake() {
        return intake;
    }

    public boolean shouldWaitClose() {
        return waitClose;
    }

    public boolean shouldWaitFull() {
        return waitFull;
    }
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
    public static final List<SuperstructureStates> safeStates = List.of(NEUTRAL, CORAL_GROUND, HANDOFF,
        ALGAE_2, ALGAE_BARGE, PRE_L3, PRE_L4, PRE_L3_BACK, PRE_L4_BACK, L2, L3, L4, L3_BACK, L4_BACK);

    public static final List<SuperstructureStates> hazardStates = List.of(START, HELD_NEUTRAL, CORAL_GROUND_L1, CORAL_LOLLIPOP, OUTTAKE,
        ALGAE_GROUND, ALGAE_LOLLIPOP, ALGAE_1, PRE_L2, L2, CLIMB);

    public static final List<Pair<SuperstructureStates, SuperstructureStates>> coupledStates = List.of(
        Pair.of(PRE_L2, L2), Pair.of(PRE_L3, L3), Pair.of(PRE_L4, L4),
        Pair.of(PRE_L3_BACK, L3_BACK), Pair.of(PRE_L4_BACK, L4_BACK)
    );

}
