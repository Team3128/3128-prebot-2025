package frc.team3128.subsystems.Arm;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import static frc.team3128.subsystems.Arm.ArmStates.*;

import java.util.List;
import java.util.function.Function;

public class Arm extends FSMSubsystemBase<ArmStates> {
    
    private static Arm instance;

    public PivotMechanism pivot;
    public RollerMechanism roller;

    private static TransitionMap<ArmStates> transitionMap = new TransitionMap<ArmStates>(ArmStates.class);
    private static final Command defaultTransitions[] = new Command[ArmStates.values().length];
    private Function<ArmStates, Command> defaultTransitioner = state -> {
        if (defaultTransitions[state.ordinal()] == null) {
            defaultTransitions[state.ordinal()] = sequence(
                pivot.pidTo(state.getAngle()),
                waitUntil(() -> pivot.atSetpoint()),
                roller.runCommand(state.getPower())
            );
        }
        return defaultTransitions[state.ordinal()];
    };

    public Arm() {
        super(ArmStates.class, transitionMap, NEUTRAL);

        pivot = PivotMechanism.getInstance();
        roller = RollerMechanism.getInstance();

        addMechanisms(roller);
        registerTransitions();
    }

    public static synchronized Arm getInstance() {
        if (instance == null) instance = new Arm();
        return instance;
    }



	@Override
	public void registerTransitions() {
        transitionMap.addCommutativeTransition(List.of(ArmStates.values()), defaultTransitioner);
	}



}