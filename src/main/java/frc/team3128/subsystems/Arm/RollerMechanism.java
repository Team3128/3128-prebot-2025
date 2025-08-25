package frc.team3128.subsystems.Arm;

import common.core.subsystems.VoltageSubsystemBase;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_TalonFX;

import static frc.team3128.Constants.ArmConstants.*;

public class RollerMechanism extends VoltageSubsystemBase {

    private static RollerMechanism instance;

    protected static NAR_TalonFX leader = new NAR_TalonFX(ROLLER_ID);

    private RollerMechanism() {
        super(leader);
    }

    public static RollerMechanism getInstance() {
        if (instance == null) {
            instance = new RollerMechanism();
        }
        return instance;
    }

    @Override
    protected void configMotors() {
        MotorConfig motorConfig = new MotorConfig(
                ROLLER_GEAR_RATIO,
                ROLLER_SAMPLE_PER_MINUTE,
                ROLLER_STATOR_CURRENT_LIMIT,
                ROLLER_INVERT,
                ROLLER_NEUTRAL_MODE,
                ROLLER_STATUS_FRAME);

        leader.configMotor(motorConfig);
    }

    @Override
    public void initShuffleboard() {
        
    }
}