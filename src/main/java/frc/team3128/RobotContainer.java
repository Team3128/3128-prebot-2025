package frc.team3128;


import common.hardware.camera.Camera;
import common.hardware.input.NAR_XboxController;
import common.hardware.input.NAR_XboxController.XboxButton;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.hardware.motorcontroller.NAR_Motor.Neutral;

import static common.hardware.input.NAR_XboxController.XboxButton.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;

import common.utility.Log;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Climber.Climber;
import frc.team3128.subsystems.Climber.ClimberStates;
import frc.team3128.subsystems.Arm.*;
// import frc.team3128.subsystems.Intake.PivotMechanism;
// import frc.team3128.subsystems.Intake.RollerMechanism;
import frc.team3128.subsystems.Elevator.*;
import frc.team3128.subsystems.Intake.*;
import frc.team3128.subsystems.Superstructure.Superstructure;

import static frc.team3128.subsystems.Superstructure.SuperstructureStates.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import frc.team3128.Constants.*;
import frc.team3128.Constants.VisionConstants.*;
import frc.team3128.Constants.FieldConstants.*;



/**
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {

    // Create all subsystems

    public static NAR_XboxController controller, controller2;

    private NarwhalDashboard dashboard;

    Arm arm;
    Climber climber;
    Elevator elevator;
    Intake intake;
    Superstructure superstructure;
    Swerve swerve;

    public RobotContainer() {
        NAR_CANSpark.maximumRetries = 2;
        NAR_TalonFX.maximumRetries = 2;

        arm = Arm.getInstance();
        climber = Climber.getInstance();
        elevator = Elevator.getInstance();
        intake = Intake.getInstance();
        superstructure = Superstructure.getInstance();
        swerve = Swerve.getInstance();

        NAR_Shuffleboard.WINDOW_WIDTH = 10;

        controller = new NAR_XboxController(2);
        controller2 = new NAR_XboxController(3);

        var swerveDriveCommand = Swerve.getInstance().getDriveCommand(controller::getLeftX, controller::getLeftY, controller::getRightX);
        CommandScheduler.getInstance().setDefaultCommand(Swerve.getInstance(), swerveDriveCommand);
        

        initCameras();
        initDashboard();
        configureButtonBindings();
    }   

    // private void configureButtonBindings3() {
    //     controller.getButton(kLeftBumper).whileTrue(Swerve.getInstance().sysIdDynamic(Direction.kForward).beforeStarting(Commands.runOnce(() -> Swerve.getInstance().zeroLock())));
    //     controller.getButton(kLeftTrigger).whileTrue(Swerve.getInstance().sysIdDynamic(Direction.kReverse).beforeStarting(Commands.runOnce(() -> Swerve.getInstance().zeroLock())));
    //     controller.getButton(kRightBumper).whileTrue(Swerve.getInstance().sysIdQuasistatic(Direction.kForward).beforeStarting(Commands.runOnce(() -> Swerve.getInstance().zeroLock())));
    //     controller.getButton(kRightTrigger).whileTrue(Swerve.getInstance().sysIdQuasistatic(Direction.kReverse).beforeStarting(Commands.runOnce(() -> Swerve.getInstance().zeroLock())));
    // }

    boolean l1Mode = false, l2Mode = false;
    private void configureButtonBindings() {
        new Trigger(() -> superstructure.getState() == NEUTRAL).debounce(0.5)
            .and(() -> arm.roller.hasObjectPresent() && arm.pivot.atSetpoint())
            .onTrue(superstructure.setStateCommand(HELD_NEUTRAL));

        controller.getUpPOVButton()
            .onTrue(Commands.runOnce(() -> Swerve.getInstance().resetGyro(0)));
        controller.getDownPOVButton()
            .onTrue(superstructure.setStateCommand(NEUTRAL));
        controller.getLeftPOVButton()
            .onTrue(arm.roller.runCommand(0.5))
            .onFalse(arm.roller.stopCommand());

        controller.getButton(kLeftTrigger)
            .onTrue(either(
                superstructure.setStateCommand(CORAL_GROUND_L1),
                superstructure.setStateCommand(CORAL_GROUND),
                () -> l1Mode
            ))
            .onFalse(either(
                superstructure.setStateCommand(HELD_NEUTRAL),
                sequence(
                    superstructure.setStateCommand(NEUTRAL),
                    waitSeconds(0.5),
                    superstructure.setStateCommand(HANDOFF),
                    waitSeconds(1),
                    superstructure.setStateCommand(NEUTRAL)
                ), 
                () -> l1Mode
            ));
        controller.getButton(kLeftBumper)
            .onTrue(superstructure.setStateCommand(OUTTAKE).onlyIf(() -> l1Mode))
            .onFalse(superstructure.setStateCommand(HELD_NEUTRAL));
        
        controller.getButton(kB)
            .onTrue(superstructure.tempToggle(PRE_L2, L2));
        controller.getButton(kX)
            .onTrue(either(
                superstructure.tempToggle(PRE_L3, L3),
                either(
                    superstructure.tempToggle(PRE_L3, L3),
                    superstructure.tempToggle(PRE_L3_BACK, L3_BACK),
                    () -> swerve.shouldScoreForward()
                ),
                () -> l2Mode
            ));
        controller.getButton(kY)
            .onTrue(either(
                superstructure.tempToggle(PRE_L4, L4),
                either(
                    superstructure.tempToggle(PRE_L4, L4),
                    superstructure.tempToggle(PRE_L4_BACK, L4_BACK),
                    () -> swerve.shouldScoreForward()
                ),
                () -> l2Mode
            ));
        
        controller.getButton(kBack)
            .onTrue(either(
                superstructure.alignScoreCoral(false),
                either(
                    superstructure.alignScoreCoral(false),
                    superstructure.alignScoreCoralBack(false),
                    () -> swerve.shouldScoreForward()
                ),
                () -> l2Mode
            ));

        controller.getButton(kStart)
            .onTrue(either(
                superstructure.alignScoreCoral(true),
                either(
                    superstructure.alignScoreCoral(true),
                    superstructure.alignScoreCoralBack(true),
                    () -> swerve.shouldScoreForward()
                ),
                () -> l2Mode
            ));

        controller.getButton(kRightTrigger)
            .onTrue(superstructure.toggle(ALGAE_1));
        controller.getButton(kRightTrigger)
            .onTrue(superstructure.toggle(ALGAE_2));

        controller2.getButton(kA)
            .onTrue(runOnce(() -> l1Mode = !l1Mode));
        controller2.getButton(kB)
            .onTrue(runOnce(() -> l2Mode = !l2Mode));

        new Trigger(() -> l1Mode)
            .onTrue(superstructure.setStateCommand(HELD_NEUTRAL))
            .onFalse(superstructure.setStateCommand(NEUTRAL));

        controller2.getButton(kX)
            .onTrue(climber.runCommand(0.4))
            .onFalse(climber.runCommand(0));

        controller.getRightPOVButton().whileTrue(swerve.driveBackwards());
    }

    private boolean lastRight = false;
    // TO BE USED WHEN ALL SUBSYSTEMS ARE READY
    private void configureButtonBindings2() {
        new Trigger(() -> superstructure.stateEquals(NEUTRAL) && arm.roller.hasObjectPresent())
            .onTrue(superstructure.setStateCommand(HELD_NEUTRAL));

        // INTAKE
        controller.getButton(kLeftTrigger)
            .onTrue(superstructure.toggle(CORAL_GROUND)
                .andThen(sequence(
                    superstructure.setStateCommand(HANDOFF),
                    waitSeconds(1),
                    superstructure.setStateCommand(NEUTRAL)
                ).onlyIf(() -> superstructure.stateEquals(NEUTRAL))
            ));
        controller.getButton(kLeftBumper)
            .onTrue(superstructure.setStateCommand(OUTTAKE))
            .onFalse(superstructure.setStateCommand(NEUTRAL));

        // ELEVATOR
        controller.getButton(kA)
            .onTrue(superstructure.tempToggle(PRE_L1, L1));
        controller.getButton(kB)
            .onTrue(superstructure.tempToggle(PRE_L2, L2));
        controller.getButton(kX)
            .onTrue(either(
                superstructure.tempToggle(PRE_L3, L3),
                superstructure.tempToggle(PRE_L3_BACK, L3_BACK)
                    .andThen(superstructure.alignScoreCoralBack(lastRight).onlyIf(() -> Swerve.autoMoveEnabled)),
                () -> swerve.shouldScoreForward()
            ));
        controller.getButton(kY)
            .onTrue(either(
                superstructure.tempToggle(PRE_L4, L4),
                superstructure.tempToggle(PRE_L4_BACK, L4_BACK)
                    .andThen(superstructure.alignScoreCoralBack(lastRight).onlyIf(() -> Swerve.autoMoveEnabled)),
                () -> swerve.shouldScoreForward()
            ));
        

        // AUTO ALIGN
        controller.getButton(kBack)
            .onTrue(superstructure.alignScoreCoral(false)
                .andThen(runOnce(() -> lastRight = false)));
        controller.getButton(kStart)
            .onTrue(superstructure.alignScoreCoral(true)
                .andThen(runOnce(() -> lastRight = true)));
    }

    public void initCameras() {
        Camera.setResources(() -> Swerve.getInstance().getYaw(), (pose, time) -> Swerve.getInstance().addVisionMeasurement(pose, time), new AprilTagFieldLayout(VisionConstants.APRIL_TAGS, FieldConstants.FIELD_X_LENGTH, FieldConstants.FIELD_Y_LENGTH), () -> Swerve.getInstance().getPose());
        //Camera.addIgnoredTags(4, 5, 14, 15);

        
        //Camera intakeCamera = new Camera("INTAKE_CAMERA", -0.30, -0.17,  -90, 0, 0);
        //intakeCamera.setThresholds(0.3, 3, 0.3);
            
        Camera backTagCamera = new Camera("BACK_TAG", Units.inchesToMeters(9), -Units.inchesToMeters(7), Units.degreesToRadians(10), -Units.degreesToRadians(10), 0);
        backTagCamera.setThresholds(0, 3, 0.3);

        Camera frontLeftCamera = new Camera("FRONT_LEFT", -Units.inchesToMeters(11), -Units.inchesToMeters(10), Units.degreesToRadians(153), 0, 0);
        frontLeftCamera.setThresholds(0, 3, 0.3);
    }

    public void initDashboard() {
        dashboard = NarwhalDashboard.getInstance();
        dashboard.addUpdate("robotX", ()-> Swerve.getInstance().getPose().getX());
        dashboard.addUpdate("robotY", ()-> Swerve.getInstance().getPose().getY());
        dashboard.addUpdate("robotYaw", ()-> Swerve.getInstance().getYaw());
    }
}
