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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
            Pose2d ogPose = Swerve.getInstance().nearestPose2d(allianceFlip(setpoints));

            if (Math.abs(swerve.getAngleTo(ogPose.getRotation())) > (Math.PI)/2) {
                Translation2d newTrans = ogPose.getTranslation().plus(MANIPULATOR_OFFSET.rotateBy(ogPose.getRotation()));
                return new Pose2d(newTrans, ogPose.getRotation().plus(Rotation2d.k180deg));
            } else {
                return ogPose;
            }
            
            // if (Math.abs(swerve.getAngleTo(tmp.getRotation())) <= (Math.PI)/2) {
            //     return(X);
            // }
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
    
    public void autoScore() {


        if (getState() == SuperstructureStates.NEUTRAL) 
            return;

        coupledStates.forEach((Pair<SuperstructureStates, SuperstructureStates> coupledState) -> {
            if (coupledState.getFirst() == getState()) {
                sequence(
                    waitUntil(() -> ElevatorMechanism.getInstance().atSetpoint()),
                    setStateCommand(coupledState.getSecond()),
                    waitSeconds(0.5),
                    setStateCommand(NEUTRAL)
                ).schedule();
            }
        });
    }

    public Command alignAlgaeIntake(Supplier<Pose2d> pose) {
        return parallel(
            swerve.navigateTo(pose),
            Commands.runOnce(
                ()-> {
                    if(FieldStates.idOf(allianceFlip(pose).get()) % 2 == 0) setStateCommand(ALGAE_2).schedule();
                    else setStateCommand(ALGAE_1).schedule();
                }
            )
        );
    }

    public Command alignCoralIntake() {
        final List<Pose2d> setpoints = List.of(FieldStates.SOURCE_LEFT.getPose2d(), FieldStates.SOURCE_RIGHT.getPose2d());
        Supplier<Pose2d> pose = ()-> swerve.nearestPose2d(allianceFlip(setpoints));
        return parallel(
            swerve.navigateTo(pose),
            setStateCommand(NEUTRAL)
        );
    }

    public Command alignAlgaeScore() {
        Supplier<Pose2d> pose = ()-> allianceFlip(new Pose2d(new Translation2d(7.6, swerve.getPose().getY()), Rotation2d.fromDegrees(0)));
        return alignAlgaeScore(pose);
    }
    
    public Command alignAlgaeScore(Supplier<Pose2d> pose) {
        return parallel(
            swerve.navigateTo(pose),
            sequence(
                waitUntil(()-> swerve.atElevatorDist()), // wait until safe for elevator to move
                setStateCommand(PRE_ALGAE_BARGE),
                Commands.runOnce(()-> delayTransition = false),
                waitUntil(() -> ElevatorMechanism.getInstance().atSetpoint()),
                waitUntil(()-> !Swerve.autoMoveEnabled),
                setStateCommand(ALGAE_BARGE),
                waitSeconds(0.5),
                setStateCommand(NEUTRAL)
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
