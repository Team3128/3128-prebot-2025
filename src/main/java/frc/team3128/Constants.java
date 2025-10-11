package frc.team3128;

import static frc.team3128.Constants.VisionConstants.APRIL_TAGS;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.hardware.motorcontroller.NAR_Motor.StatusFrames;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
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

        public static final List<AprilTag> APRIL_TAGS = Arrays.asList(
            new AprilTag(1, new Pose3d(Units.inchesToMeters(657.37), Units.inchesToMeters(25.80), Units.inchesToMeters(58.50), new Rotation3d(0, Math.toRadians(0), Math.toRadians(126)))),
            new AprilTag(2, new Pose3d(Units.inchesToMeters(657.37), Units.inchesToMeters(291.20), Units.inchesToMeters(58.50), new Rotation3d(0, Math.toRadians(0), Math.toRadians(234)))),
            new AprilTag(3, new Pose3d(Units.inchesToMeters(455.15), Units.inchesToMeters(317.15), Units.inchesToMeters(51.25), new Rotation3d(0, Math.toRadians(0), Math.toRadians(270)))),
            new AprilTag(4, new Pose3d(Units.inchesToMeters(365.20), Units.inchesToMeters(241.64), Units.inchesToMeters(73.54), new Rotation3d(0, Math.toRadians(30), Math.toRadians(0)))),
            new AprilTag(5, new Pose3d(Units.inchesToMeters(265.20), Units.inchesToMeters(75.39), Units.inchesToMeters(73.54), new Rotation3d(0, Math.toRadians(30), Math.toRadians(0)))),
            new AprilTag(6, new Pose3d(Units.inchesToMeters(530.49), Units.inchesToMeters(130.17), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(300)))),
            new AprilTag(7, new Pose3d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.50), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(0)))),
            new AprilTag(8, new Pose3d(Units.inchesToMeters(530.49), Units.inchesToMeters(186.83), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(60)))),
            new AprilTag(9, new Pose3d(Units.inchesToMeters(497.77), Units.inchesToMeters(186.83), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(120)))),
            new AprilTag(10, new Pose3d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.50), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(180)))),
            new AprilTag(11, new Pose3d(Units.inchesToMeters(497.77), Units.inchesToMeters(130.17), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(240)))),
            new AprilTag(12, new Pose3d(Units.inchesToMeters(33.51), Units.inchesToMeters(25.80), Units.inchesToMeters(58.50), new Rotation3d(0, Math.toRadians(0), Math.toRadians(54)))),
            new AprilTag(13, new Pose3d(Units.inchesToMeters(33.51), Units.inchesToMeters(291.20), Units.inchesToMeters(58.50), new Rotation3d(0, Math.toRadians(0), Math.toRadians(306)))),
            new AprilTag(14, new Pose3d(Units.inchesToMeters(325.68), Units.inchesToMeters(241.64), Units.inchesToMeters(73.54), new Rotation3d(0, Math.toRadians(30), Math.toRadians(180)))),
            new AprilTag(15, new Pose3d(Units.inchesToMeters(325.68), Units.inchesToMeters(75.39), Units.inchesToMeters(73.54), new Rotation3d(0, Math.toRadians(30), Math.toRadians(180)))),
            new AprilTag(16, new Pose3d(Units.inchesToMeters(235.73), Units.inchesToMeters(-0.15), Units.inchesToMeters(51.25), new Rotation3d(0, Math.toRadians(0), Math.toRadians(90)))),
            new AprilTag(17, new Pose3d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(240)))),
            new AprilTag(18, new Pose3d(Units.inchesToMeters(144.00), Units.inchesToMeters(158.50), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(180)))),
            new AprilTag(19, new Pose3d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(120)))),
            new AprilTag(20, new Pose3d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(60)))),
            new AprilTag(21, new Pose3d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.58), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(0)))),
            new AprilTag(22, new Pose3d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(300))))
        );

    }
    
    public static class FieldConstants {
        public static final double FIELD_X_LENGTH = Units.inchesToMeters(690.875); // meters = 17.548
        public static final double FIELD_Y_LENGTH = Units.inchesToMeters(317); // meters = 8.052
        public static final Translation2d FIELD = new Translation2d(FIELD_X_LENGTH, FIELD_Y_LENGTH);
        public static final Translation2d CENTER_FIELD = FIELD.div(2);
        public static final Translation2d MANIP_OFFSET = new Translation2d(Units.inchesToMeters(29.0 / 2.0), Units.inchesToMeters(8));
        public static final Translation2d MANIP_OFFSET_BACK = new Translation2d(Units.inchesToMeters(29.0 / 2.0), Units.inchesToMeters(-8));
        public static final Translation2d REEF_STOP_DIST = new Translation2d(Units.inchesToMeters(5.0), 0);
        public static final Translation2d CORAL_LEFT_POLE_SHIFT = new Translation2d(0, Units.inchesToMeters(-13.0 / 2));
        public static final Translation2d SOURCE_LEFT_SHIFT = new Translation2d(0, Units.inchesToMeters(-12.5));

        public enum FieldStates {
            A(18, REEF_STOP_DIST.plus(CORAL_LEFT_POLE_SHIFT)),
            B(18, REEF_STOP_DIST.minus(CORAL_LEFT_POLE_SHIFT)),
            C(17, REEF_STOP_DIST.plus(CORAL_LEFT_POLE_SHIFT)),
            D(17, REEF_STOP_DIST.minus(CORAL_LEFT_POLE_SHIFT)),
            E(22, REEF_STOP_DIST.plus(CORAL_LEFT_POLE_SHIFT)),
            F(22, REEF_STOP_DIST.minus(CORAL_LEFT_POLE_SHIFT)),
            G(21, REEF_STOP_DIST.plus(CORAL_LEFT_POLE_SHIFT)),
            H(21, REEF_STOP_DIST.minus(CORAL_LEFT_POLE_SHIFT)),
            I(20, REEF_STOP_DIST.plus(CORAL_LEFT_POLE_SHIFT)),
            J(20, REEF_STOP_DIST.minus(CORAL_LEFT_POLE_SHIFT)),
            K(19, REEF_STOP_DIST.plus(CORAL_LEFT_POLE_SHIFT)),
            L(19, REEF_STOP_DIST.minus(CORAL_LEFT_POLE_SHIFT)),

            ALGAE_AB(18, REEF_STOP_DIST),
            ALGAE_CD(17, REEF_STOP_DIST),
            ALGAE_EF(22, REEF_STOP_DIST),
            ALGAE_GH(21, REEF_STOP_DIST),
            ALGAE_IJ(20, REEF_STOP_DIST),
            ALGAE_KL(19, REEF_STOP_DIST);

            private final int id;
            private final Pose2d pose;
            private final Pose2d backPose;

            private FieldStates(int id, Translation2d offset) {
                this.id = id;
                final Pose2d aprilTagPose = APRIL_TAGS.get(id - 1).pose.toPose2d();
                final Translation2d fieldOffset = offset.rotateBy(aprilTagPose.getRotation());
                final Translation2d fieldManipOffset = MANIP_OFFSET.rotateBy(aprilTagPose.getRotation());
                final Translation2d fieldManipOffsetBack = MANIP_OFFSET_BACK.rotateBy(aprilTagPose.getRotation());
                final Rotation2d rotation = aprilTagPose.getRotation().plus(Rotation2d.k180deg);
                final Rotation2d rotationBack = aprilTagPose.getRotation();
                this.pose = new Pose2d(aprilTagPose.getTranslation().plus(fieldManipOffset).plus(fieldOffset), rotation);
                this.backPose = new Pose2d(aprilTagPose.getTranslation().plus(fieldManipOffsetBack).plus(fieldOffset), rotationBack);
            }

            public Pose2d getPose2d() {
                return this.pose;
            }

            public Pose2d getBackPose2d() {
                return this.backPose;
            }

            public Translation2d getTranslation2d() {
                return pose.getTranslation();
            }

            public Rotation2d getRotation2d() {
                return pose.getRotation();
            }

            public int getId() {
                return this.id;
            }

            public static int idOf(Pose2d pose) {
                for(FieldStates state : FieldStates.values()) {
                    if(pose.equals(state.getPose2d())) return state.getId();
                }
                return -1;
            }

            public static final List<FieldStates> coral = List.of(A, B, C, D, E, F, G, H, I, J, K, L);
            public static final List<FieldStates> coralLeft = List.of(A, C, F, H, J, K);
            public static final List<FieldStates> coralRight = List.of(B, D, E, G, I, L);
            public static final List<FieldStates> algae = List.of(ALGAE_AB, ALGAE_CD, ALGAE_EF, ALGAE_GH, ALGAE_IJ, ALGAE_KL);
        }
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

        public static final double WINCH_GEAR_RATIO = 1; //9.0 / 360.0;
        public static final double WINCH_SAMPLE_PER_MINUTE = 60;
        public static final int WINCH_STATOR_CURRENT_LIMIT = 40;
        public static final boolean WINCH_INVERT = true;
        public static final Neutral WINCH_NEUTRAL_MODE = Neutral.BRAKE;
        public static final StatusFrames WINCH_STATUS_FRAME = StatusFrames.POSITION;

        public static final double WINCH_POSITION_MIN = 0;
        public static final double WINCH_POSITION_MAX = 75;
        public static final double WINCH_TOLERANCE = 0.4;

        public static final double ROLLER_GEAR_RATIO = 1;
        public static final double ROLLER_SAMPLE_PER_MINUTE = 60;
        public static final int ROLLER_STATOR_CURRENT_LIMIT = 40;
        public static final boolean ROLLER_INVERT = false;
        public static final Neutral ROLLER_NEUTRAL_MODE = Neutral.BRAKE;
        public static final StatusFrames ROLLER_STATUS_FRAME = StatusFrames.POSITION;

    }

    public static class ElevatorConstants {

        public static final int BOTTOM_ID = 40;
        public static final int TOP_ID = 41;

        public static final double GEAR_RATIO = 1.3905 / 27.15;//Units.inchesToMeters(60.4375) / 40.18735;
        public static final double SAMPLE_PER_MINUTE = 60;
        public static final int STATOR_CURRENT_LIMIT = 60;
        public static final boolean INVERT = true;
        public static final Neutral NEUTRAL_MODE = Neutral.BRAKE;
        public static final StatusFrames STATUS_FRAME = StatusFrames.POSITION;

        public static final double POSITION_MIN = 0;
        public static final double POSITION_MAX = 1.3905;
        public static final double TOLERANCE = 0.01;

    }

    public static class IntakeConstants {
    
        public static final int PIVOT_ID = 50;

        public static final double PIVOT_GEAR_RATIO = 360.0 / 30.0;
        public static final double PIVOT_SAMPLE_PER_MINUTE = 60;
        public static final int PIVOT_STATOR_CURRENT_LIMIT = 40;
        public static final boolean PIVOT_INVERT = false;
        public static final Neutral PIVOT_NEUTRAL_MODE = Neutral.BRAKE;
        public static final StatusFrames PIVOT_STATUS_FRAME = StatusFrames.POSITION;

        public static final double PIVOT_POSITION_MIN = 0;
        public static final double PIVOT_POSITION_MAX = 135;
        public static final double PIVOT_TOLERANCE = 1;

        public static final int ROLLER_ID = 51;

        public static final double ROLLER_GEAR_RATIO = 1;
        public static final double ROLLER_SAMPLE_PER_MINUTE = 60;
        public static final int ROLLER_STATOR_CURRENT_LIMIT = 40;
        public static final boolean ROLLER_INVERT = false;
        public static final Neutral ROLLER_NEUTRAL_MODE = Neutral.BRAKE;
        public static final StatusFrames ROLLER_STATUS_FRAME = StatusFrames.POSITION;

    }
    public static class ArmConstants {
    
        public static final int PIVOT_ID = 20;

        public static final double PIVOT_GEAR_RATIO = 360 * (0.2 * 1.0 / 3.0 * 14.0 / 68.0);
        public static final double PIVOT_SAMPLE_PER_MINUTE = 60;
        public static final int PIVOT_STATOR_CURRENT_LIMIT = 40;
        public static final boolean PIVOT_INVERT = true;
        public static final Neutral PIVOT_NEUTRAL_MODE = Neutral.BRAKE;
        public static final StatusFrames PIVOT_STATUS_FRAME = StatusFrames.POSITION;

        public static final double PIVOT_POSITION_MIN = -360;
        public static final double PIVOT_POSITION_MAX = 360;
        public static final double PIVOT_TOLERANCE = 1;

        public static final int ROLLER_ID = 21;

        public static final double ROLLER_GEAR_RATIO = 1;
        public static final double ROLLER_SAMPLE_PER_MINUTE = 60;
        public static final int ROLLER_STATOR_CURRENT_LIMIT = 30;
        public static final boolean ROLLER_INVERT = true;
        public static final Neutral ROLLER_NEUTRAL_MODE = Neutral.BRAKE;
        public static final StatusFrames ROLLER_STATUS_FRAME = StatusFrames.POSITION;
        
        public static final String CAM_OBD = ""; // change this based on the object 
        public static final double X_OFFSET = 0;
        public static final double Y_OFFSET = 0;
        public static final double YAW_OFFSET = 0;
        public static final double PITCH_OFFSET = 0;
        public static final double ROLL_OFFSET = 0;

        public static final double INTAKE_HEIGHT = 0; // TODO
        public static final double ARM_LENGTH = 0; // TODO

        public static final double CURRENT_THRESHOLD = 0.0; // TODO
    }

    public static class SuperstructureConstants {
        public static final double PIVOT_SAFE_ANGLE = 0.0; // TODO
        public static final double ELEVATOR_SAFE_HEIGHT = 0.9; // TODO
    }

}