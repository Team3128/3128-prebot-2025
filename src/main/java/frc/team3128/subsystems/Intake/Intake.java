package frc.team3128.subsystems.Intake;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import static frc.team3128.subsystems.Intake.IntakeStates.*;

import java.util.List;
import java.util.function.Function;

public class Intake extends FSMSubsystemBase<IntakeStates> {
    
    private static Intake instance;

    protected PivotMechanism pivot;
    protected RollerMechanism roller;

    private static TransitionMap<IntakeStates> transitionMap = new TransitionMap<IntakeStates>(IntakeStates.class);
    private Function<IntakeStates, Command> defaultTransitioner = state -> {
        return sequence(
            pivot.pidTo(state.getAngle()),
            waitUntil(() -> pivot.atSetpoint()),
            roller.runCommand(state.getPower())
        );
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
	}
}