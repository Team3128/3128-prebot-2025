package frc.team3128.subsystems.Climber;

import common.core.controllers.Controller;
import common.core.controllers.ControllerBase;
import common.core.controllers.PIDFFConfig;
import common.core.subsystems.PositionSubsystemBase;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;

import static frc.team3128.Constants.ClimberConstants.*;

public class WinchMechanism extends PositionSubsystemBase {

    private static WinchMechanism instance;

    private static PIDFFConfig config = new PIDFFConfig(0.00001, 0, 0, 12, 0, 0, 0);
    protected static ControllerBase controller = new Controller(config, Controller.Type.POSITION);

    protected static NAR_TalonFX leader = new NAR_TalonFX(WINCH_ID);

    private WinchMechanism() {
        super(controller, leader);
        initShuffleboard();
    }

    public static WinchMechanism getInstance() {
        if (instance == null) {
            instance = new WinchMechanism();
        }
        return instance;
    }

    @Override
    protected void configMotors() {
        MotorConfig motorConfig = new MotorConfig(
                WINCH_GEAR_RATIO,
                WINCH_SAMPLE_PER_MINUTE,
                WINCH_STATOR_CURRENT_LIMIT,
                WINCH_INVERT,
                WINCH_NEUTRAL_MODE,
                WINCH_STATUS_FRAME);

        leader.configMotor(motorConfig);
    }

    @Override
    protected void configController() {
        controller.setInputRange(WINCH_POSITION_MIN, WINCH_POSITION_MAX);
        controller.configureFeedback(leader);
        controller.setTolerance(WINCH_TOLERANCE);
    }

}