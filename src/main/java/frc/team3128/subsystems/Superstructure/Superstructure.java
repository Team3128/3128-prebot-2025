package frc.team3128.subsystems.Superstructure;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static frc.team3128.subsystems.Superstructure.SuperstructureStates.*;

import frc.team3128.Constants.FieldConstants.FieldStates;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Arm.Arm;
import frc.team3128.subsystems.Elevator.Elevator;
import frc.team3128.subsystems.Intake.Intake;

import static frc.team3128.Constants.FieldConstants.*;

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
                elevator.setStateCommand(state.getElevator()),
                intake.setStateCommand(state.getIntake())
            ).beforeStarting(waitUntil(() -> !swerve.shouldWaitClose()).onlyIf(() -> state.shouldWaitClose()))
            .beforeStarting(waitUntil(() -> !swerve.shouldWaitFull()).onlyIf(() -> state.shouldWaitFull()));;
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
                waitUntil(() -> arm.pivot.closeToSetpoint()),
                elevator.setStateCommand(state.getElevator())
            ).beforeStarting(waitUntil(() -> !swerve.shouldWaitClose()).onlyIf(() -> state.shouldWaitClose()))
            .beforeStarting(waitUntil(() -> !swerve.shouldWaitFull()).onlyIf(() -> state.shouldWaitFull()));
        }
        return toHazardTransitions[state.ordinal()];
    };
    private static final Command fromHazardTransitions[] = new Command[SuperstructureStates.values().length];
    private Function<SuperstructureStates, Command> fromHazardTransitioner = state -> {
        if (fromHazardTransitions[state.ordinal()] == null) {
            fromHazardTransitions[state.ordinal()] = sequence(
                parallel(
                    elevator.setStateCommand(state.getElevator()),
                    intake.setStateCommand(state.getIntake())
                ),
                waitUntil(() -> elevator.elevator.closeToSetpoint()),
                arm.setStateCommand(state.getArm())
            ).beforeStarting(waitUntil(() -> !swerve.shouldWaitClose()).onlyIf(() -> state.shouldWaitClose()))
            .beforeStarting(waitUntil(() -> !swerve.shouldWaitFull()).onlyIf(() -> state.shouldWaitFull()));
        }
        return fromHazardTransitions[state.ordinal()];
    };

    private Superstructure() {
        super(SuperstructureStates.class, transitionMap, START);

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
        transitionMap.addDivergingTransition(START, (Command) null);
        transitionMap.addTransition(START, NEUTRAL, fromHazardTransitioner);
    }

    public Command toggle(Command defaultCommand, Command exclusiveCommand, BooleanSupplier condition) {
        return either(
            exclusiveCommand,
            defaultCommand,
            condition
        );
    }

    public Command toggle(SuperstructureStates defaultState, SuperstructureStates exclusiveState, BooleanSupplier condition) {
        return toggle(setStateCommand(defaultState), setStateCommand(exclusiveState), condition);
    }

    public Command toggle(SuperstructureStates defaultState, SuperstructureStates exclusiveState) {
        return toggle(defaultState, exclusiveState, ()-> stateEquals(defaultState));
    }

    public Command toggle(SuperstructureStates state) {
        return toggle(state, NEUTRAL);
    }

    public Command tempToggle(SuperstructureStates defaultState, SuperstructureStates exclusiveState, BooleanSupplier condition, double delay) {
        return either(
            sequence(
                setStateCommand(exclusiveState),
                waitSeconds(0.5),
                setStateCommand(NEUTRAL)
            ),
            setStateCommand(defaultState), 
            condition
        );
    }

    public Command tempToggle(SuperstructureStates defaultStates, SuperstructureStates exclusiveState, BooleanSupplier condition) {
        return tempToggle(defaultStates, exclusiveState, condition, 1);
    }

    public Command tempToggle(SuperstructureStates defaultStates, SuperstructureStates exclusiveState) {
        return tempToggle(defaultStates, exclusiveState,  ()-> stateEquals(defaultStates), 1);
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
        Supplier<Pose2d> pose = () -> allianceFlip(swerve.nearest(fieldStates).getPose2d());
        return alignScoreCoral(pose);
    }

    public Command alignScoreCoralBack(boolean isRight) {
        final List<FieldStates> fieldStates = isRight ? FieldStates.coralRight : FieldStates.coralLeft;
        Supplier<Pose2d> pose = () -> allianceFlip(swerve.nearest(fieldStates).getBackPose2d());
        return alignScoreCoral(pose);
    }
}
