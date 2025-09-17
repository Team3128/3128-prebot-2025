package frc.team3128;


import common.hardware.input.NAR_XboxController;
import common.hardware.input.NAR_XboxController.XboxButton;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_TalonFX;

import static common.hardware.input.NAR_XboxController.XboxButton.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;

import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

        controller.getButton(kA).whileTrue(Commands.runOnce(()->Swerve.getInstance().drive(new Translation2d(1,0), 0)));
        controller.getButton(kA).whileTrue(Commands.runOnce(()->Swerve.getInstance().drive(new Translation2d(0,0), 1)));
        controller.getButton(kB).onTrue((Arm.getInstance().roller.runCommand(0.1)));
        controller.getButton(kX).onTrue((Elevator.getInstance().elevator.runCommand(0.1)));
        controller.getButton(kY).onTrue((Intake.getInstance().pivot.runCommand(0.1)));
        controller.getButton(kBack).onTrue((Intake.getInstance().roller.runCommand(0.1)));

        controller.getButton(kLeftBumper).whileTrue(Swerve.getInstance().sysIdDynamic(Direction.kForward).beforeStarting(Commands.runOnce(()->Swerve.getInstance().zeroLock())));
        controller.getButton(kLeftTrigger).whileTrue(Swerve.getInstance().sysIdDynamic(Direction.kReverse).beforeStarting(Commands.runOnce(()->Swerve.getInstance().zeroLock())));
        controller.getButton(kRightBumper).whileTrue(Swerve.getInstance().sysIdQuasistatic(Direction.kForward).beforeStarting(Commands.runOnce(()->Swerve.getInstance().zeroLock())));
        controller.getButton(kRightTrigger).whileTrue(Swerve.getInstance().sysIdQuasistatic(Direction.kReverse).beforeStarting(Commands.runOnce(()->Swerve.getInstance().zeroLock())));
    }

    public void initCameras() {

    }

    public void initDashboard() {

    }
}
