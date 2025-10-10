package frc.team3128.subsystems.Superstructure;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static frc.team3128.Constants.SuperstructureConstants.*;
import static frc.team3128.subsystems.Superstructure.SuperstructureStates.*;

import frc.team3128.Constants.FieldConstants.FieldStates;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Arm.Arm;
import frc.team3128.subsystems.Elevator.Elevator;
import frc.team3128.subsystems.Intake.Intake;

public class Superstructure extends FSMSubsystemBase<SuperstructureStates> {

    private static Superstructure instance;

    private static Arm arm;
    private static Elevator elevator;
    private static Intake intake;
    private static Swerve swerve;

    private static TransitionMap<SuperstructureStates> transitionMap = new TransitionMap<>(SuperstructureStates.class);
    private static final Command defaultTransitions[] = new Command[SuperstructureStates.values().length];
    private Function<SuperstructureStates, Command> defaultTransitioner = state -> {
        if (defaultTransitions[state.ordinal()] == null) {
            defaultTransitions[state.ordinal()] = parallel(
                arm.setStateCommand(state.getArm()),
                elevator.setStateCommand(state.getElevator())
                    .beforeStarting(waitUntil(() -> !swerve.shouldWait()).onlyIf(() -> state.shouldWait())),
                intake.setStateCommand(state.getIntake())
            );
        }
        return defaultTransitions[state.ordinal()];
    };
    private static final Command toHazardTransitions[] = new Command[SuperstructureStates.values().length];
    private Function<SuperstructureStates, Command> toHazardTransitioner = state -> {
        if (toHazardTransitions[state.ordinal()] == null) {
            toHazardTransitions[state.ordinal()] = sequence(
                parallel(
                    arm.setStateCommand(state.getArm()),
                    intake.setStateCommand(state.getIntake())
                ),
                waitUntil(() -> Math.abs(MathUtil.inputModulus(arm.pivot.getPosition(), -180, 180)) >= PIVOT_SAFE_ANGLE),
                elevator.setStateCommand(state.getElevator())
                    .beforeStarting(waitUntil(() -> !swerve.shouldWait()).onlyIf(() -> state.shouldWait()))
            );
        }
        return toHazardTransitions[state.ordinal()];
    };
    private static final Command fromHazardTransitions[] = new Command[SuperstructureStates.values().length];
    private Function<SuperstructureStates, Command> fromHazardTransitioner = state -> {
        if (fromHazardTransitions[state.ordinal()] == null) {
            fromHazardTransitions[state.ordinal()] = sequence(
                parallel(
                    elevator.setStateCommand(state.getElevator())
                        .beforeStarting(waitUntil(() -> !swerve.shouldWait()).onlyIf(() -> state.shouldWait())),
                    intake.setStateCommand(state.getIntake())
                ),
                waitUntil(() -> elevator.elevator.getPosition() >= ELEVATOR_SAFE_HEIGHT),
                arm.setStateCommand(state.getArm())
            );
        }
        return fromHazardTransitions[state.ordinal()];
    };

    private Superstructure() {
        super(SuperstructureStates.class, transitionMap, SuperstructureStates.NEUTRAL);

        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        intake = Intake.getInstance();
        swerve = Swerve.getInstance();

        registerTransitions();
        NAR_Shuffleboard.addData("Superstructure", "state", () -> this.getState().name());
    }

    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    @Override
    public void registerTransitions() {
        transitionMap.addCommutativeTransition(SuperstructureStates.safeStates, defaultTransitioner);
        transitionMap.addConvergingTransition(SuperstructureStates.hazardStates, toHazardTransitioner);
        transitionMap.addDivergingTransition(SuperstructureStates.hazardStates, fromHazardTransitioner);
    }

    public Command toggle(SuperstructureStates state1, SuperstructureStates state2) {
        return either(setStateCommand(state1), setStateCommand(state2), () -> stateEquals(state2));
    }

    public Command toggle(SuperstructureStates state) {
        return toggle(state, SuperstructureStates.NEUTRAL);
    }

    public Command tempToggle(SuperstructureStates state1, SuperstructureStates state2, double delay) {
        return either(
            sequence(
                setStateCommand(state2),
                waitSeconds(delay),
                setStateCommand(NEUTRAL)
            ),
            setStateCommand(state1),
            () -> stateEquals(state1)
        );
    }

    public Command tempToggle(SuperstructureStates state1, SuperstructureStates state2) {
        return tempToggle(state1, state2, 0.5);
    }

    public Command alignScoreCoral(Supplier<Pose2d> pose) {
        return sequence(
            swerve.navigateTo(pose),
            Commands.runOnce(() -> {
                for (Pair<SuperstructureStates, SuperstructureStates> coupledState : coupledStates) {
                    if (stateEquals(coupledState.getFirst())) {
                        sequence(
                            setStateCommand(coupledState.getSecond()),
                            waitSeconds(0.5),
                            setStateCommand(NEUTRAL)
                        ).schedule();
                        break;
                    }
                }
            })
        );
    }

    public Command alignScoreCoral(boolean isRight) {
        final List<FieldStates> fieldStates = isRight ? FieldStates.coralRight : FieldStates.coralLeft;
        Supplier<Pose2d> pose = () -> swerve.nearest(fieldStates).getPose2d();
        return alignScoreCoral(pose);
    }

    public Command alignScoreCoralBack(boolean isRight) {
        final List<FieldStates> fieldStates = isRight ? FieldStates.coralRight : FieldStates.coralLeft;
        Supplier<Pose2d> pose = () -> swerve.nearest(fieldStates).getBackPose2d();
        return alignScoreCoral(pose);
    }

    // public Command alignIntakeAlgae()
}
