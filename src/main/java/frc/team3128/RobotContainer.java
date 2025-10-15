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

    private void configureButtonBindings() {
        new Trigger(() -> superstructure.getState() == NEUTRAL).debounce(0.5)
            .and(() -> arm.roller.hasObjectPresent() && arm.pivot.atSetpoint())
            .onTrue(superstructure.setStateCommand(HELD_NEUTRAL));
        Trigger t = new Trigger(() -> superstructure.getState() == NEUTRAL).debounce(0.5);
        NAR_Shuffleboard.addData("Auto Align", "Closest", () -> swerve.nearest(FieldStates.coralRight).name(), 0, 1);
        NAR_Shuffleboard.addData("Auto Align", "Should Forward", () -> swerve.shouldScoreForward(), 1, 1);
        // controller.getButton(kA).onTrue(Swerve.getInstance().identifyOffsetsCommand().ignoringDisable(true));
        controller.getUpPOVButton().onTrue(Commands.runOnce(() -> Swerve.getInstance().resetGyro(0)));

        // controller.getButton(kA).whileTrue(Commands.run(()->Swerve.getInstance().drive(new Translation2d(0.2,0), 0)));
        // controller.getButton(kB).whileTrue(Commands.run(()->Swerve.getInstance().drive(new Translation2d(0,0), 0.2)));
        // controller.getButton(kX).onTrue(ElevatorMechanism.getInstance().runCommand(0.4)).onFalse(PivotMechanism.getInstance().stopCommand());
        // controller.getButton(kY).onTrue(ElevatorMechanism.getInstance().runCommand(-0.4)).onFalse(PivotMechanism.getInstance().stopCommand());
        controller.getDownPOVButton().onTrue(superstructure.setStateCommand(NEUTRAL));
        // controller.getButton(kA).onTrue(ElevatorMechanism.getInstance().runCommand(0.4)).onFalse(ElevatorMechanism.getInstance().stopCommand());
        // controller.getButton(kB).onTrue(ElevatorMechanism.getInstance().runCommand(-0.4)).onFalse(ElevatorMechanism.getInstance().stopCommand());
        // controller.getButton(kA).onTrue(Arm.getInstance().pivot.resetCommand(0));
        // controller.getButton(kLeftBumper).onTrue(ElevatorMechanism.getInstance().pidTo(0.3));
        // controller.getButton(kRightBumper).onTrue(ElevatorMechanism.getInstance().pidTo(1.1));
        // controller.getButton(kBack).whileTrue((Elevator.getInstance().elevator.runCommand(0.2))).onFalse(Elevator.getInstance().elevator.stopCommand());
        // controller.getButton(kStart).whileTrue((Elevator.getInstance().elevator.runCommand(-0.2))).onFalse(Elevator.getInstance().elevator.stopCommand());
        // controller.getButton(kY).onTrue((Intake.getInstance().pivot.runCommand(0.1)));
        // controller.getButton(kBack).onTrue((Intake.getInstance().roller.runCommand(0.1)));

        controller.getButton(kLeftTrigger).onTrue(superstructure.setStateCommand(CORAL_GROUND)).onFalse(superstructure.setStateCommand(NEUTRAL));
        controller.getButton(kLeftBumper).onTrue(superstructure.setStateCommand(OUTTAKE)).onFalse(superstructure.setStateCommand(NEUTRAL));
        controller.getButton(kA).onTrue(superstructure.setStateCommand(HANDOFF)).onFalse(superstructure.setStateCommand(NEUTRAL));
        // controller.getButton(kX).onTrue(arm.pivot.resetCommand(180));
        controller.getButton(kB).onTrue(superstructure.toggle(L2, PRE_L2));
        controller.getButton(kX).and(controller.getButton(kRightTrigger)).onTrue(superstructure.toggle(L3, PRE_L3));
        controller.getButton(kX).and(controller.getButton(kRightBumper)).onTrue(superstructure.toggle(L3_BACK, PRE_L3_BACK));
        // controller.getButton(kX).onTrue(superstructure.toggle(L3, PRE_L3));
        controller.getButton(kY).and(controller.getButton(kRightTrigger)).onTrue(superstructure.toggle(L4, PRE_L4));
        controller.getButton(kY).and(controller.getButton(kRightBumper)).onTrue(superstructure.toggle(L4_BACK, PRE_L4_BACK));
        controller.getButton(kBack).onTrue(
            either(
                superstructure.alignScoreCoral(false),
                superstructure.alignScoreCoralBack(false),
                () -> swerve.shouldScoreForward()
            )
        );
        controller.getButton(kStart).onTrue(
            either(
                superstructure.alignScoreCoral(true),
                superstructure.alignScoreCoralBack(true),
                () -> swerve.shouldScoreForward()
            )
        );
        // controller.getButton(kY).onTrue(superstructure.toggle(L4, PRE_L4));

        // controller.getButton(kY).onTrue(superstructure.setStateCommand(ALGAE_1)).onFalse(superstructure.setStateCommand(HELD_NEUTRAL));
        // controller.getButton(kB).onTrue(superstructure.setStateCommand(HELD_NEUTRAL));
        // controller.getButton(kX).onTrue(superstructure.setStateCommand(ALGAE_2)).onFalse(superstructure.setStateCommand(HELD_NEUTRAL));

        // controller.getButton(kBack).onTrue(arm.pivot.runCommand(0.4)).onFalse(arm.pivot.runCommand(-0));
        // controller.getButton(kStart).onTrue(arm.pivot.runCommand(-0.4)).onFalse(arm.pivot.runCommand(-0));
        // controller.getButton(kLeftBumper).whileTrue(ElevatorMechanism.getInstance().sysIdDynamic(Direction.kForward));
        // controller.getButton(kLeftTrigger).whileTrue(ElevatorMechanism.getInstance().sysIdDynamic(Direction.kReverse));
        // controller.getButton(kRightBumper).whileTrue(ElevatorMechanism.getInstance().sysIdQuasistatic(Direction.kForward));
        // controller.getButton(kRightTrigger).whileTrue(ElevatorMechanism.getInstance().sysIdQuasistatic(Direction.kReverse));

        // controller.getButton(kLeftBumper).whileTrue(Swerve.getInstance().sysIdDynamic(Direction.kForward).beforeStarting(Commands.runOnce(()->Swerve.getInstance().zeroLock())));
        // controller.getButton(kLeftTrigger).whileTrue(Swerve.getInstance().sysIdDynamic(Direction.kReverse).beforeStarting(Commands.runOnce(()->Swerve.getInstance().zeroLock())));
        // controller.getButton(kRightBumper).whileTrue(Swerve.getInstance().sysIdQuasistatic(Direction.kForward).beforeStarting(Commands.runOnce(()->Swerve.getInstance().zeroLock())));
        // controller.getButton(kRightTrigger).whileTrue(Swerve.getInstance().sysIdQuasistatic(Direction.kReverse).beforeStarting(Commands.runOnce(()->Swerve.getInstance().zeroLock())));
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
            
        Camera backTagCamera = new Camera("BACK_TAG", Units.inchesToMeters(8.875), -Units.inchesToMeters(7), Units.degreesToRadians(10), -Units.degreesToRadians(10), 0);
        backTagCamera.setThresholds(0, 3, 0.3);

        Camera frontLeftCamera = new Camera("FRONT_LEFT", -0.27, -0.27, Units.degreesToRadians(153), 0, 0);
        frontLeftCamera.setThresholds(0, 3, 0.3);
    }

    public void initDashboard() {
        dashboard = NarwhalDashboard.getInstance();
        dashboard.addUpdate("robotX", ()-> Swerve.getInstance().getPose().getX());
        dashboard.addUpdate("robotY", ()-> Swerve.getInstance().getPose().getY());
        dashboard.addUpdate("robotYaw", ()-> Swerve.getInstance().getYaw());
    }
}
