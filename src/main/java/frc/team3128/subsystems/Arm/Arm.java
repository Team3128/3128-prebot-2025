package frc.team3128.subsystems.Arm;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.team3128.Constants.ArmConstants.*;
import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import common.hardware.camera.Camera;

import static frc.team3128.subsystems.Arm.ArmStates.*;

import java.util.List;
import java.util.function.Function;

public class Arm extends FSMSubsystemBase<ArmStates> {
    
    private static Arm instance;

    public PivotMechanism pivot;
    public RollerMechanism roller;
    public Camera cam;

    

    private static TransitionMap<ArmStates> transitionMap = new TransitionMap<ArmStates>(ArmStates.class);
    private static final Command defaultTransitions[] = new Command[ArmStates.values().length];
    private Function<ArmStates, Command> defaultTransitioner = state -> {
        if (defaultTransitions[state.ordinal()] == null) {
            if (state == NEUTRAL || state == L1 || state == L2 || state == L3 || state == L4) {
                defaultTransitions[state.ordinal()] = sequence(
                    roller.stopCommand(),
                    pivot.pidTo(state.getAngle()),
                    waitUntil(() -> pivot.atSetpoint()),
                    roller.runCommand(state.getPower())
                );
            } else {
                defaultTransitions[state.ordinal()] = sequence(
                    roller.runCommand(-0.6),
                    pivot.pidTo(state.getAngle()),
                    waitUntil(() -> pivot.atSetpoint()),
                    roller.runCommand(state.getPower())
                );
            }
        }
        return defaultTransitions[state.ordinal()];
    };

    public Arm() {
        super(ArmStates.class, transitionMap, START);

        pivot = PivotMechanism.getInstance();
        roller = RollerMechanism.getInstance();
        cam = new Camera(CAM_OBD, X_OFFSET, Y_OFFSET, YAW_OFFSET, PITCH_OFFSET, ROLL_OFFSET);

        addMechanisms(roller);
        registerTransitions();
    }

    public static synchronized Arm getInstance() {
        if (instance == null) instance = new Arm();
        return instance;
    }

    private static double findPivotAngle(double camYaw) {
        final double xOffset = INTAKE_HEIGHT * Math.tan(Math.toRadians(camYaw));
        final double pivotAngle = Math.toDegrees(Math.asin(xOffset / ARM_LENGTH));
        return pivotAngle;
    }

	@Override
	public void registerTransitions() {
        transitionMap.addCommutativeTransition(List.of(ArmStates.values()), defaultTransitioner);
        transitionMap.addConvergingTransition(HANDOFF, sequence(
            roller.runCommand(-0.3),
            pivot.pidTo(findPivotAngle(0.0)),
            roller.runCommand(HANDOFF.getPower())
        ));
	}
}