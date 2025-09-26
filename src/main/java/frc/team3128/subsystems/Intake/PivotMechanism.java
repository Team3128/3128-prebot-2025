package frc.team3128.subsystems.Intake;

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
import static frc.team3128.Constants.IntakeConstants.*;

import static edu.wpi.first.units.Units.*;

public class PivotMechanism extends PositionSubsystemBase {

    private static PivotMechanism instance;

    private static PIDFFConfig config = new PIDFFConfig(0.16, 0, 0, 0.23783, 0.01558, 0.00234, 0.0);

    protected static Controller controller = new Controller(config, Controller.Type.POSITION);

    protected static NAR_TalonFX leader = new NAR_TalonFX(PIVOT_ID);

    private PivotMechanism() {
        super(controller, leader);
        initShuffleboard();

        leader.setUnitConversionFactor(PIVOT_GEAR_RATIO);
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

    public SysIdRoutine driveRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.2).per(Second), Volts.of(3), null),
        new SysIdRoutine.Mechanism((v) -> runVolts(v.in(Volts)), this::logMotors, this)
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return driveRoutine.quasistatic(direction);
    }
      
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return driveRoutine.dynamic(direction);
    }
    
    private final MutVoltage appliedVoltage = Volts.mutable(0);
    private final MutAngle angle = Degrees.mutable(0);
    private final MutAngularVelocity velocity = DegreesPerSecond.mutable(0);
    public void logMotors(SysIdRoutineLog log){
        log.motor("pivot-leader")
            .angularPosition(angle.mut_replace(leader.getPosition(), Degrees))
            .angularVelocity(velocity.mut_replace(leader.getVelocity(), DegreesPerSecond))
            .voltage(appliedVoltage.mut_replace(leader.getMotor().getMotorVoltage().getValueAsDouble(), Volts));
    }

}