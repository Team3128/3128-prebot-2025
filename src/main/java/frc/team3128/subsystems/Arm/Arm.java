package frc.team3128.subsystems.Arm;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.RobotContainer;
import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import common.hardware.limelight.Limelight;
import common.hardware.limelight.LimelightKey;
import common.hardware.camera.Camera;

import static frc.team3128.subsystems.Arm.ArmStates.*;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import frc.team3128.Constants.ArmConstants;

public class Arm extends FSMSubsystemBase<ArmStates> {
    
    private static Arm instance;

    protected PivotMechanism pivot;
    protected RollerMechanism roller;
    public Camera cam; 

    private static TransitionMap<ArmStates> transitionMap = new TransitionMap<ArmStates>(ArmStates.class);
    private Function<ArmStates, Command> defaultTransitioner = state -> {
        return sequence(
            pivot.pidTo(state.getAngle()),
            waitUntil(() -> pivot.atSetpoint()),
            roller.runCommand(state.getPower())
        );
    };

    public Arm() {
        super(ArmStates.class, transitionMap, NEUTRAL);

        pivot = PivotMechanism.getInstance();
        roller = RollerMechanism.getInstance();

        addMechanisms(roller);
        registerTransitions();
        cam = new Camera(ArmConstants.CAM_OBD, ArmConstants.X_OFFSET, ArmConstants.Y_OFFSET, ArmConstants.YAW_OFFSET, ArmConstants.PITCH_OFFSET, ArmConstants.ROLL_OFFSET);
    }

    public static double findPivotAngle(double horizontalOffset) {
            return Math.asin((horizontalOffset/67)*(180/Math.PI)); // TODO: replace 67 with arm length
    } 
    public static double findPhotonPivotAngle(double yaw){
        return yaw;
    }

    public static synchronized Arm getInstance() {
        if (instance == null) instance = new Arm();
        return instance;
    }

	@Override
	public void registerTransitions() {
        transitionMap.addCommutativeTransition(List.of(ArmStates.values()), defaultTransitioner);
        transitionMap.addConvergingTransition(HANDOFF, sequence(
            // pivot.pidTo(findPivotAngle(RobotContainer.limelight.getValue(LimelightKey.HORIZONTAL_OFFSET))),  // this is for limelight 
            pivot.pidTo(findPhotonPivotAngle(cam.getYaw()), 
            waitUntil(() -> pivot.atSetpoint()),
            roller.runCommand(HANDOFF.getPower())
        ));
	}
} 