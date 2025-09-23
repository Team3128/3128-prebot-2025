package frc.team3128.subsystems.Arm;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import frc.team3128.subsystems.Arm.Arm;

public class ArmTest {
    private Arm armSubsystem;

    @BeforeEach
    public void setUp() {
        armSubsystem = Arm.getInstance();
    }

    @Test
    public void testPivotMechanismPidTo() {
        double targetAngle = 45.0;
        Command command = armSubsystem.pivot.pidTo(targetAngle);

        assertNotNull(command, "The command for pivot PID should not be null.");
        // Assuming the command modifies the pivot mechanism's state
        // You can verify the pivot mechanism's state or behavior here
    }

    @Test
    public void testRollerMechanismRunCommand() {
        double targetPower = 0.5;
        Command command = armSubsystem.roller.runCommand(targetPower);

        assertNotNull(command, "The command for roller run should not be null.");
        // Assuming the command modifies the roller mechanism's state
        // You can verify the roller mechanism's state or behavior here
    }

    // @Test
    // public void testTransitionToState() {
    //     ArmStates targetState = ArmStates.SOME_STATE; // Replace with an actual state
    //     Command transitionCommand = armSubsystem.transitionTo(targetState);

    //     assertNotNull(transitionCommand, "The transition command should not be null.");
    //     // Assuming the transition command modifies the subsystem's state
    //     // You can verify the subsystem's state or behavior here
    // }

    @Test
    public void testAtSetpoint() {
        // Assuming the pivot mechanism has a method to check if it's at the setpoint
        boolean atSetpoint = armSubsystem.pivot.atSetpoint();

        // Replace with the expected behavior
        assertFalse(atSetpoint, "The pivot mechanism should not be at the setpoint initially.");
    }
}