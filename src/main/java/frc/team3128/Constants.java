package frc.team3128;

import java.util.ArrayList;
import java.util.List;

import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.hardware.motorcontroller.NAR_Motor.StatusFrames;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
public class Constants {

    public static class DriveConstants {

        public static final double controllerPOVOffset = -90;

        public static final double slow = 0.3;
        public static final double medium = 0.6;
        public static final double fast = 1;

        /* Swerve Profiling Values */
        // Theoretical: v = 4.96824, omega = 11.5
        // Real: v = 4.5, omega = 10
        // For safety, use less than theoretical and real values
        public static final double MAX_DRIVE_SPEED = 4.5;//4.8; //meters per second - 16.3 ft/sec
        public static final double MAX_ATTAINABLE_DRIVE_SPEED = MAX_DRIVE_SPEED; //Stole from citrus.
        public static final double MAX_DRIVE_ACCELERATION = 3.4;//5;
        public static final double MAX_DRIVE_ANGULAR_VELOCITY = 2 * Math.PI;//10
        public static final double MAX_DRIVE_ANGULAR_ACCELERATION = 10;//2 * Math.PI; //I stole from citrus.

        public static final double driveMotorGearRatio = 0;
        public static final double angleMotorGearRatio = 150.0 / 7; 

        // public static final var kinematics = null; 

        /* Motor Current Limiting */
        public static final int angleMotorCurrentLimit = 0;
        public static final int driveMotorCurrentLimit = 0;
        public static final int motorStallCurrentLimit = 0;

        /* Angle Motor PID Values */
        public static final double angleKP = 0;
        public static final double angleKI = 0;
        public static final double angleKD = 0;
        public static final double angleKF = 0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0;
        public static final double driveKI = 0;
        public static final double driveKD = 0;
        public static final double driveKF = 0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0;
        public static final double driveKV = 0;
        public static final double driveKA = 0;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        public static final MotorConfig driveMotorConfig = null;

        public static final MotorConfig angleMotorConfig = null;

        public static final PIDFFConfig drivePIDConfig = new PIDFFConfig(driveKP, driveKI, driveKD, driveKS, driveKV, driveKA);

        public static final PIDFFConfig anglePIDConfig = new PIDFFConfig(angleKP, angleKI, angleKD);

        public static final List<Rotation2d> snapToAngles = new ArrayList<>();
        static {
            snapToAngles.add(Rotation2d.fromDegrees(-180));
            snapToAngles.add(Rotation2d.fromDegrees(-90));
            snapToAngles.add(Rotation2d.fromDegrees(0));
            snapToAngles.add(Rotation2d.fromDegrees(90));
            snapToAngles.add(Rotation2d.fromDegrees(180));
        }
    }

    public static class VisionConstants {

        public static final Matrix<N3,N1> SVR_STATE_STD = VecBuilder.fill(0.1, 0.1,Units.degreesToRadians(3));
 
        public static final Matrix<N3,N1> SVR_VISION_MEASUREMENT_STD = VecBuilder.fill(0.5,0.5,Units.degreesToRadians(5));

    }
    public static class SwerveConstants {
        /* Module Device IDs */
        public static final int MOD0_DRIVE_MOTOR_ID = 1;
        public static final int MOD0_ANGLE_MOTOR_ID = 2;
        public static final int MOD0_CANCODER_ID = 10;
        public static final int MOD1_DRIVE_MOTOR_ID = 3;
        public static final int MOD1_ANGLE_MOTOR_ID = 4;
        public static final int MOD1_CANCODER_ID = 11;
        public static final int MOD2_DRIVE_MOTOR_ID = 5;
        public static final int MOD2_ANGLE_MOTOR_ID = 6;
        public static final int MOD2_CANCODER_ID = 12;
        public static final int MOD3_DRIVE_MOTOR_ID = 7;
        public static final int MOD3_ANGLE_MOTOR_ID = 8;
        public static final int MOD3_CANCODER_ID = 13;

        /* Cancoder Offsets */
        public static final double MOD0_CANCODER_OFFSET = 40.95703125;
        public static final double MOD1_CANCODER_OFFSET = 80.50781249999999;
        public static final double MOD2_CANCODER_OFFSET = -6.064453125;
        public static final double MOD3_CANCODER_OFFSET = -174.638671875;
        // public static final double MOD0_CANCODER_OFFSET = -117.94921874999999;
        // public static final double MOD1_CANCODER_OFFSET = -68.90625;
        // public static final double MOD2_CANCODER_OFFSET = 66.796875;
        // public static final double MOD3_CANCODER_OFFSET = 24.609375;



        public static final double RAMP_TIME = 3;

        /* Drivetrain Constants */
        public static final double ROBOT_MASS = 62; //kg
        public static final double WHEEL_COF = 1.2;
        public static final double DRIVE_BUMPER_LENGTH = Units.inchesToMeters(5);
        public static final double DRIVE_TRACK_WIDTH = Units.inchesToMeters(20.75); //Hand measure later
        public static final double DRIVE_WHEEL_BASE = Units.inchesToMeters(20.75); //Hand measure later
        public static final double ROBOT_LENGTH = Units.inchesToMeters(26.5) + DRIVE_BUMPER_LENGTH; // bumperLength + trackWidth;
        public static final double DRIVE_WHEEL_DIAMETER = 0.0486 * 2;
        public static final double DRIVE_WHEEL_CIRCUMFERENCE = (DRIVE_WHEEL_DIAMETER * Math.PI);
        public static final double ROBOT_MOI = ROBOT_MASS * (DRIVE_TRACK_WIDTH / 2) * 0.44965 / 0.4443; //kg m^2 mass * (trackWidth / 2) * (Ka angular / Ka linear)

        public static final double closedLoopRamp = 0.0;

        public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
        public static final double DRIVE_ANGLE_GEAR_RATIO = (150.0 / 7.0); // 300.0 / 13.0

        /* Swerve Current Limiting */
        public static final int DRIVE_ANGLE_CURRENT_LIMIT = 30; //30
        public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60; //40;

        /* Angle Motor PID Values */
        // switched 364 pid values to SDS pid values
        public static final double DRIVE_ANGLE_KP = 0.15 * 30; // 0.6; // citrus: 0.3 //0.15
        public static final double DRIVE_ANGLE_KI = 0.0;
        public static final double DRIVE_ANGLE_KD = 0.0; // 12.0; // citrus: 0
        public static final double DRIVE_ANGLE_KF = 0.0;

        /* Drive Motor PID Values */
        public static final double DRIVE_MOTOR_KP = 4e-5; //4e-5, //0.05
        public static final double DRIVE_MOTOR_KI = 0.0;
        public static final double DRIVE_MOTOR_KD = 0.0;
        public static final double DRIVE_MOTOR_KF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double DRIVE_MOTOR_KS = 0.16621;//0.16746;//0.13023; //0.19057;//0.60094; // 0.19225;
        public static final double DRIVE_MOTOR_KV = 2.62229; //1.95619;//1.92348; //2.01208;//1.1559;  // 2.4366
        public static final double DRIVE_MOTOR_KA = -0.42513; // 0.4443;//0.10274; //0.09043; //0.12348; // 0.34415

        /* Motor and Sensor IDs */
        public static final int PIDGEON_ID = 9; 
        public static final String DRIVETRAIN_CANBUS_NAME = "drivetrain";
        public static final double TRANSLATIONAL_DEADBAND = 0.5;
        public static final double ROTATIONAL_DEADBAND = 0.1;
        /* Motor Inverts */
        public static final boolean DRIVE_MOTOR_INVERTED = false;
        public static final boolean DRIVE_ANGLE_INVERTED = true;

        /* Angle Encoder Invert */
        public static final boolean ANGLE_CANCODER_INVERTED = false;

        public static final double DRIVE_TURN_KP = 5;
        public static final double DRIVE_TURN_KI = 0;
        public static final double DRIVE_TURN_KD = 0;
        public static final double DRIVE_TURN_KS = 0.09545; //, 0.1 , 0.05748
        public static final double DRIVE_TURN_KV = 2.64897; //0.01723
        public static final double DRIVE_TURN_KA = -0.11944; //0.0064
        public static final double DRIVE_TURN_KG = 0;

        public static final List<Rotation2d> snapToAngles = List.of(
            Rotation2d.fromDegrees(-180),
            Rotation2d.fromDegrees(-120),
            Rotation2d.fromDegrees(-60),
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(60),
            Rotation2d.fromDegrees(120),
            Rotation2d.fromDegrees(180)
        );
    }

    public static class ClimberConstants {

        public static final int WINCH_ID = 30;
        public static final int ROLLER_ID = 31;

        public static final double WINCH_GEAR_RATIO = 1.0; // TODO
        public static final double WINCH_SAMPLE_PER_MINUTE = 60;
        public static final int WINCH_STATOR_CURRENT_LIMIT = 40;
        public static final boolean WINCH_INVERT = false; // TODO
        public static final Neutral WINCH_NEUTRAL_MODE = Neutral.BRAKE;
        public static final StatusFrames WINCH_STATUS_FRAME = StatusFrames.POSITION;

        public static final double WINCH_POSITION_MIN = 0;
        public static final double WINCH_POSITION_MAX = 180;
        public static final double WINCH_TOLERANCE = 1;

        public static final double ROLLER_GEAR_RATIO = 1;
        public static final double ROLLER_SAMPLE_PER_MINUTE = 60;
        public static final int ROLLER_STATOR_CURRENT_LIMIT = 40;
        public static final boolean ROLLER_INVERT = true;
        public static final Neutral ROLLER_NEUTRAL_MODE = Neutral.BRAKE;
        public static final StatusFrames ROLLER_STATUS_FRAME = StatusFrames.POSITION;

    }

    public static class ElevatorConstants {

        public static final int LEFT_ID = 40;
        public static final int RIGHT_ID = 41;

        public static final double GEAR_RATIO = Units.inchesToMeters(60.4375) / 40.18735;
        public static final double SAMPLE_PER_MINUTE = 60;
        public static final int STATOR_CURRENT_LIMIT = 60;
        public static final boolean INVERT = false;
        public static final Neutral NEUTRAL_MODE = Neutral.COAST;
        public static final StatusFrames STATUS_FRAME = StatusFrames.POSITION;

        public static final double POSITION_MIN = 0;
        public static final double POSITION_MAX = Units.inchesToMeters(60.4375);
        public static final double TOLERANCE = 0.01;

    }
}