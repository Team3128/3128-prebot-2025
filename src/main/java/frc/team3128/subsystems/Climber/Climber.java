package frc.team3128.subsystems.Climber;

import java.util.List;
import java.util.function.Function;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;

import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import static frc.team3128.subsystems.Climber.ClimberStates.*;

public class Climber extends FSMSubsystemBase<ClimberStates> {
    
    private static Climber instance;

    public WinchMechanism winch;
    public RollerMechanism roller;

    private static TransitionMap<ClimberStates> transitionMap = new TransitionMap<ClimberStates>(ClimberStates.class);
    private static final Command defaultTransitions[] = new Command[ClimberStates.values().length];
    private Function<ClimberStates, Command> defaultTransitioner = state -> {
        if (defaultTransitions[state.ordinal()] == null) {
            defaultTransitions[state.ordinal()] = sequence(
                roller.stopCommand(),
                runOnce(() -> WinchMechanism.controller.getConfig().kS = () -> 12 * state.getWinchPower()),
                winch.pidTo(state.getAngle()),
                roller.runCommand(state.getRollerPower())
            );
        }
        return defaultTransitions[state.ordinal()];
    };

    public static synchronized Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public Climber() {
        super(ClimberStates.class, transitionMap, NEUTRAL);

        winch = WinchMechanism.getInstance();
        roller = RollerMechanism.getInstance();
        
        addMechanisms(winch, roller);

        registerTransitions();
    }

    @Override
    public void registerTransitions() {
        transitionMap.addCommutativeTransition(List.of(ClimberStates.values()), defaultTransitioner);
    }

}
