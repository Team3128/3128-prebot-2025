package frc.team3128.subsystems.Elevator;

import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.subsystems.PositionSubsystemBase;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.utility.shuffleboard.NAR_Shuffleboard;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_CANSpark.ControllerType;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static frc.team3128.Constants.ElevatorConstants.*;

import static edu.wpi.first.units.Units.*;

public class ElevatorMechanism extends PositionSubsystemBase {

    private static ElevatorMechanism instance;
    //30, 0, 0, 0.25086, 4.52908, 0.99630, 0
    private static PIDFFConfig config = new PIDFFConfig(1.5227, 0, 0, 0.16967, 4.3806, 0.3332, 0.264);
    protected static Controller controller = new Controller(config, Controller.Type.POSITION);

    protected static NAR_CANSpark left = new NAR_CANSpark(BOTTOM_ID, ControllerType.CAN_SPARK_FLEX), right = new NAR_CANSpark(TOP_ID,ControllerType.CAN_SPARK_FLEX);

    private ElevatorMechanism() {
        super(controller, left, right);
        left.setUnitConversionFactor(GEAR_RATIO);
        right.setUnitConversionFactor(GEAR_RATIO);
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
    public void initShuffleboard() {
        super.initShuffleboard();
        NAR_Shuffleboard.addData(getName(), "Position", () -> left.getPosition());
        NAR_Shuffleboard.addData(getName(), "Velocity", () -> left.getVelocity());
        NAR_Shuffleboard.addData(getName(), "Voltage", () -> left.getAppliedOutput() * 12);
    }

    @Override
    protected void configController() {
       controller.setInputRange(POSITION_MIN, POSITION_MAX);
       controller.configureFeedback(left);
       controller.setTolerance(TOLERANCE);
    }
    public SysIdRoutine driveRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.2).per(Second), Volts.of(4), null),
        new SysIdRoutine.Mechanism((v) -> runVolts(v.in(Volts)), this::logMotors, this)
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return driveRoutine.quasistatic(direction).onlyWhile(() -> left.getPosition() > POSITION_MIN + 0.2*(POSITION_MAX - POSITION_MIN) && left.getPosition() < POSITION_MAX-0.2*(POSITION_MAX - POSITION_MIN));
    }
      
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return driveRoutine.dynamic(direction).onlyWhile(() -> left.getPosition() > POSITION_MIN + 0.2*(POSITION_MAX - POSITION_MIN) && left.getPosition() < POSITION_MAX-0.2*(POSITION_MAX - POSITION_MIN));
    }
    
    private final MutVoltage appliedVoltage = Volts.mutable(0);
    private final MutDistance position = Meters.mutable(0);
    private final MutLinearVelocity velocity = MetersPerSecond.mutable(0);
    public void logMotors(SysIdRoutineLog log){
        // log.motor("position").linearPosition(Meters.of(left.getPosition()));
        // log.motor("velocity").linearVelocity(MetersPerSecond.of(left.getVelocity() / 60.0));
        // log.motor("voltage").voltage(Volts.of(12 * left.getAppliedOutput()));
        log.motor("elevator-motor")
            .linearPosition(position.mut_replace(left.getPosition(), Meters))
            .linearVelocity(velocity.mut_replace(left.getVelocity(), MetersPerSecond))
            .voltage(appliedVoltage.mut_replace(left.getAppliedOutput() * 12, Volts));
    }
}