package frc.team3128.subsystems.Climber;

import common.core.subsystems.VoltageSubsystemBase;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_CANSpark.ControllerType;

import static frc.team3128.Constants.ClimberConstants.*;

public class RollerMechanism extends VoltageSubsystemBase {

    private static RollerMechanism instance;

    public static NAR_CANSpark leader = new NAR_CANSpark(ROLLER_ID, ControllerType.CAN_SPARK_FLEX);

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