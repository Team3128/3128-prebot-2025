package frc.team3128.subsystems.Intake;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.subsystems.Arm.Arm;
import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import static frc.team3128.subsystems.Intake.IntakeStates.*;

import java.util.List;
import java.util.function.Function;

public class Intake extends FSMSubsystemBase<IntakeStates> {
    
    private static Intake instance;

    public PivotMechanism pivot;
    public RollerMechanism roller;

    private static TransitionMap<IntakeStates> transitionMap = new TransitionMap<IntakeStates>(IntakeStates.class);
    private static final Command defaultTransitions[] = new Command[IntakeStates.values().length];
    private Function<IntakeStates, Command> defaultTransitioner = state -> {
        if (defaultTransitions[state.ordinal()] == null) {
            defaultTransitions[state.ordinal()] = sequence(
                pivot.pidTo(state.getAngle()),
                waitUntil(() -> pivot.atSetpoint()),
                roller.runCommand(state.getPower())
            );
        }
        return defaultTransitions[state.ordinal()];
    };

    public Intake() {
        super(IntakeStates.class, transitionMap, NEUTRAL);

        pivot = PivotMechanism.getInstance();
        roller = RollerMechanism.getInstance();

        addMechanisms(roller);
        registerTransitions();
    }

    public static synchronized Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }

	@Override
	public void registerTransitions() {
        transitionMap.addCommutativeTransition(List.of(IntakeStates.values()), defaultTransitioner);
        transitionMap.addConvergingTransition(HANDOFF, sequence(
            pivot.pidTo(HANDOFF.getAngle()),
            waitUntil(() -> pivot.atSetpoint() && Arm.getInstance().pivot.atSetpoint()),
            roller.runCommand(HANDOFF.getPower())
        ));
	}
}