package frc.team3128;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.Function;

import common.core.fsm.FSMSubsystemBase;

import frc.team3128.Robot;
import frc.team3128.subsystems.Arm.Arm;
import frc.team3128.subsystems.Intake.Intake;
import frc.team3128.subsystems.Elevator.Elevator;
import frc.team3128.subsystems.Superstructure.Superstructure;
import frc.team3128.subsystems.Superstructure.SuperstructureStates;

public class TestSuper {

    private Superstructure superstructure = Superstructure.getInstance();

    @Test
    @DisplayName("Demo Test") 
    void demoTest() {
        superstructure.setStateCommand(SuperstructureStates.ALGAE_1);
        assertNotNull(superstructure.getState());
        //assertEquals(a, b);
    }
    
}
