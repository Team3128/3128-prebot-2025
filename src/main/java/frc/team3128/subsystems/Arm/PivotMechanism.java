package frc.team3128.subsystems.Arm;

import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.subsystems.PositionSubsystemBase;
import common.hardware.motorcontroller.NAR_TalonFX;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;

import static frc.team3128.Constants.ArmConstants.*;

import static edu.wpi.first.units.Units.*;

public class PivotMechanism extends PositionSubsystemBase {

    private static PivotMechanism instance;

    private static PIDFFConfig config = new PIDFFConfig(0.08, 0, 0, 0.16944, 0.019594, 0.0037586, 0.10644);

    protected static Controller controller = new Controller(config, Controller.Type.POSITION);

    protected static NAR_TalonFX leader = new NAR_TalonFX(PIVOT_ID);

    private PivotMechanism() {
        super(controller, leader);
        config.kG_Function = () -> Math.sin(Math.toRadians(leader.getPosition()));
        leader.setUnitConversionFactor(PIVOT_GEAR_RATIO);
        leader.setTimeConversionFactor(60);
        initShuffleboard();

    }

    public static PivotMechanism getInstance() {
        if (instance == null) {
            instance = new PivotMechanism();
        }
        return instance;
    }

    @Override
    protected void configMotors() {
        MotorConfig motorConfig = new MotorConfig(
                PIVOT_GEAR_RATIO, 
                PIVOT_SAMPLE_PER_MINUTE,
                PIVOT_STATOR_CURRENT_LIMIT,
                PIVOT_INVERT,
                PIVOT_NEUTRAL_MODE,
                PIVOT_STATUS_FRAME);

        leader.configMotor(motorConfig);
    }

    @Override
    protected void configController() {
        controller.setInputRange(PIVOT_POSITION_MIN, PIVOT_POSITION_MAX);
       controller.configureFeedback(leader);
       controller.setTolerance(PIVOT_TOLERANCE);
    }   

    @Override
    public void initShuffleboard() {
        super.initShuffleboard();
        // NAR_Shuffleboard.addData(getName(), "Position", () -> Degrees.of(leader.getPosition()).toLongString(), 2, 5);
        // NAR_Shuffleboard.addData(getName(), "Velocity", () -> DegreesPerSecond.of(leader.getVelocity()).toLongString(), 1, 5);
        // NAR_Shuffleboard.addData(getName(), "Volts", () -> Volts.of(leader.getMotor().getMotorVoltage().getValueAsDouble()).toLongString(), 3, 5);
    }

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.3).per(Second), Volts.of(4), null),
        new SysIdRoutine.Mechanism((v) -> runVolts(v.in(Volts)), this::logMotors, this)
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }
      
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
    
    private final MutVoltage appliedVoltage = Volts.mutable(0);
    private final MutAngle angle = Degrees.mutable(0);
    private final MutAngularVelocity velocity = DegreesPerSecond.mutable(0);
    public void logMotors(SysIdRoutineLog log){
        // log.motor("position").angularPosition(Degrees.of(leader.getPosition()));
        // log.motor("velocity").angularVelocity(DegreesPerSecond.of(leader.getVelocity()));
        // log.motor("voltage").voltage(Volts.of(12 * leader.getAppliedOutput()));
        log.motor("pivot-leader")
            .angularPosition(angle.mut_replace(leader.getPosition(), Degrees))
            .angularVelocity(velocity.mut_replace(leader.getVelocity(), DegreesPerSecond))
            .voltage(appliedVoltage.mut_replace(leader.getMotor().getMotorVoltage().getValueAsDouble(), Volts));
    }

    public boolean closeToSetpoint() {
        return Math.abs(getSetpoint() - controller.getMeasurement()) <= 10 * PIVOT_TOLERANCE;
    }
    
}