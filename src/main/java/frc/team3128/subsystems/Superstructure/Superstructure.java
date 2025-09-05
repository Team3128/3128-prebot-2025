package frc.team3128.subsystems.Superstructure;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Arm.Arm;
import frc.team3128.subsystems.Elevator.Elevator;
import frc.team3128.subsystems.Elevator.ElevatorMechanism;
import frc.team3128.subsystems.Intake.Intake;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.team3128.Constants.*;
import frc.team3128.Constants.FieldConstants.FieldStates;
import static frc.team3128.Constants.FieldConstants.*;
import frc.team3128.subsystems.Superstructure.SuperstructureStates;
import static frc.team3128.subsystems.Superstructure.SuperstructureStates.*;


import java.util.List;
// import java.util.function.Supplier;


public class Superstructure extends FSMSubsystemBase<SuperstructureStates> {

    private static Superstructure instance;

    private static Arm arm;
    private static Elevator elevator;
    private static Intake intake;
    private static Swerve swerve;

    private static boolean delayTransition = false;

    private static TransitionMap<SuperstructureStates> transitionMap = new TransitionMap<>(SuperstructureStates.class);
    private static final Command defaultTransitions[] = new Command[SuperstructureStates.values().length];
    private Function<SuperstructureStates, Command> defaultTransitioner = state -> {
        if (defaultTransitions[state.ordinal()] == null) {
            defaultTransitions[state.ordinal()] = parallel(
                arm.setStateCommand(state.getArm()),
                elevator.setStateCommand(state.getElevator()),
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
                waitUntil(() -> arm.pivot.atSetpoint()),
                elevator.setStateCommand(state.getElevator())
            );
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
                waitUntil(() -> elevator.elevator.atSetpoint()),
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

        registerTransitions();
    }

    public Command alignScoreCoral(boolean isRight) {;
        final List<Pose2d> setpoints = isRight ? FieldStates.reefRight.asJava() : FieldStates.reefLeft.asJava();
        Supplier<Pose2d> pose = () -> {
            Pose2d tmp = Swerve.getInstance().nearestPose2d(allianceFlip(setpoints));
            return new Pose2d(tmp.getX(), tmp.getY(), tmp.getRotation().plus(Rotation2d.fromDegrees(Math.abs(swerve.getAngleTo(tmp.getRotation())) <= (Math.PI)/2 ? 0 : 180)));
    };
        return alignScoreCoral(pose, ()-> false, ()->true);
    }

    public Command alignScoreCoral(Supplier<Pose2d> pose, BooleanSupplier shouldRam, BooleanSupplier shouldWait){
        return parallel(
            sequence(
                Commands.runOnce(()-> delayTransition = true),
                swerve.navigateTo(pose)
            ),
            sequence(
                waitUntil(()-> swerve.atElevatorDist()), // wait until safe for elevator to move
                Commands.runOnce(()-> delayTransition = false),
                Commands.runOnce(()-> {
                    for(Pair<SuperstructureStates, SuperstructureStates> coupledState : coupledStates){
                        if (coupledState.getFirst() == getState()) {
                            sequence(
                                waitUntil(() -> ElevatorMechanism.getInstance().atSetpoint()),
                                waitUntil(()-> !Swerve.autoMoveEnabled).onlyIf(shouldWait),
                                setStateCommand(coupledState.getSecond()),
                                waitSeconds(0.5),
                                setStateCommand(NEUTRAL)
                            ).schedule();
                            return;
                        }
                    }
                })
            )
        );
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
}
