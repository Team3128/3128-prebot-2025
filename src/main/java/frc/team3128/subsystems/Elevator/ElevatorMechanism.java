package frc.team3128.subsystems.Elevator;

import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.subsystems.PositionSubsystemBase;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_TalonFX;

import static frc.team3128.Constants.ElevatorConstants.*;

public class ElevatorMechanism extends PositionSubsystemBase {

    private static ElevatorMechanism instance;
    //30, 0, 0, 0.25086, 4.52908, 0.99630, 0
    private static PIDFFConfig config = new PIDFFConfig(20, 0, 0, 0.6, 2.91916, 0.67429, 0.3);
    protected static Controller controller = new Controller(config, Controller.Type.POSITION);

    protected static NAR_TalonFX left = new NAR_TalonFX(LEFT_ID), right = new NAR_TalonFX(RIGHT_ID);

    private ElevatorMechanism() {
        super(controller, left, right);
    }

    public static synchronized ElevatorMechanism getInstance() {
        if (instance == null) instance = new ElevatorMechanism();
        return instance;
    }

    @Override
    protected void configMotors() {
        MotorConfig motorConfig = new MotorConfig(
                GEAR_RATIO, 
                SAMPLE_PER_MINUTE,
                STATOR_CURRENT_LIMIT,
                INVERT,
                NEUTRAL_MODE,
                STATUS_FRAME);

        left.configMotor(motorConfig);
        right.configMotor(motorConfig.follower());

        initShuffleboard();
    }

    @Override
    protected void configController() {
       controller.setInputRange(POSITION_MIN, POSITION_MAX);
       controller.configureFeedback(left);
       controller.setTolerance(TOLERANCE);
    }
}