package frc.team3128.subsystems.Arm;

import common.core.subsystems.VoltageSubsystemBase;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.utility.shuffleboard.NAR_Shuffleboard;
import common.hardware.motorcontroller.NAR_TalonFX;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static frc.team3128.Constants.ArmConstants.*;

import static edu.wpi.first.units.Units.*;

public class RollerMechanism extends VoltageSubsystemBase {

    private static RollerMechanism instance;

    protected static NAR_TalonFX leader = new NAR_TalonFX(ROLLER_ID);

    private RollerMechanism() {
        super(CURRENT_THRESHOLD, leader);

        leader.setUnitConversionFactor(ROLLER_GEAR_RATIO);
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
        NAR_Shuffleboard.addData(getName(), "Current", this::getCurrent, 0, 0);
        NAR_Shuffleboard.addData(getName(), "Has Object", this::hasObjectPresent, 1, 0);
    }

        public SysIdRoutine driveRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(7), null),
        new SysIdRoutine.Mechanism((v) -> runVolts(v.in(Volts)), this::logMotors, this)
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return driveRoutine.quasistatic(direction);
    }
      
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return driveRoutine.dynamic(direction);
    }
    
    public void logMotors(SysIdRoutineLog log){
        log.motor("position").angularPosition(Rotations.of(leader.getPosition()));
        log.motor("velocity").angularVelocity(RotationsPerSecond.of(leader.getVelocity() / 60.0));
        log.motor("voltage").voltage(Volts.of(12 * leader.getAppliedOutput()));
    }
}