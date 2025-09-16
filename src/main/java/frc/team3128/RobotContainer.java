package frc.team3128;


import common.hardware.input.NAR_XboxController;
import common.hardware.input.NAR_XboxController.XboxButton;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_TalonFX;

import static common.hardware.input.NAR_XboxController.XboxButton.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;

import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Arm.*;
import frc.team3128.subsystems.Elevator.*;
import frc.team3128.subsystems.Intake.*;


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

    public RobotContainer() {
        NAR_CANSpark.maximumRetries = 2;
        NAR_TalonFX.maximumRetries = 2;

        NAR_Shuffleboard.WINDOW_WIDTH = 10;

        controller = new NAR_XboxController(2);
        controller2 = new NAR_XboxController(3);

        var swerveDriveCommand = Swerve.getInstance().getDriveCommand(controller::getLeftX, controller::getLeftY, controller::getRightX);
        CommandScheduler.getInstance().setDefaultCommand(Swerve.getInstance(), swerveDriveCommand);
        

        initCameras();
        initDashboard();
        configureButtonBindings();
    }   

    private void configureButtonBindings() {
        // controller.getButton(kA).onTrue(Swerve.getInstance().identifyOffsetsCommand().ignoringDisable(true));
        controller.getUpPOVButton().onTrue(Commands.runOnce(() -> Swerve.getInstance().resetGyro(0)));

        controller.getButton(kA).onTrue((Arm.getInstance().pivot.runCommand(0.1)));
        controller.getButton(kB).onTrue((Arm.getInstance().roller.runCommand(0.1)));
        controller.getButton(kX).onTrue((Elevator.getInstance().elevator.runCommand(0.1)));
        controller.getButton(kY).onTrue((Intake.getInstance().pivot.runCommand(0.1)));
        controller.getButton(kBack).onTrue((Intake.getInstance().roller.runCommand(0.1)));

        controller.getButton(kLeftBumper).whileTrue(Arm.getInstance().pivot.sysIdDynamic(Direction.kForward));
        controller.getButton(kLeftTrigger).whileTrue(Arm.getInstance().pivot.sysIdDynamic(Direction.kReverse));
        controller.getButton(kRightBumper).whileTrue(Arm.getInstance().pivot.sysIdQuasistatic(Direction.kForward));
        controller.getButton(kRightTrigger).whileTrue(Arm.getInstance().pivot.sysIdQuasistatic(Direction.kReverse));
    }

    public void initCameras() {

    }

    public void initDashboard() {

    }
}
