// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3128;

import java.util.List;
import java.util.Optional;

import common.core.misc.NAR_Robot;
import common.core.subsystems.PositionSubsystemBase;
import common.hardware.camera.Camera;
import common.utility.Log;
import static common.utility.Log.Type.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.subsystems.Intake.IntakeStates.INTAKE;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team3128.Constants.FieldConstants.FieldStates;
import frc.team3128.autonomous.AutoPrograms;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Arm.Arm;
import frc.team3128.subsystems.Arm.ArmStates;
import frc.team3128.subsystems.Climber.Climber;
import frc.team3128.subsystems.Climber.ClimberStates;
import frc.team3128.subsystems.Elevator.Elevator;
import frc.team3128.subsystems.Elevator.ElevatorStates;
import frc.team3128.subsystems.Intake.Intake;
import frc.team3128.subsystems.Intake.IntakeStates;
import frc.team3128.subsystems.Superstructure.Superstructure;
import frc.team3128.subsystems.Superstructure.SuperstructureStates;

// import frc.team3128.autonomous.AutoPrograms;
import com.pathplanner.lib.commands.PathfindingCommand;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends NAR_Robot {

    public static Alliance alliance;

    public static Alliance getAlliance() {
        if (alliance == null) {
            Optional<Alliance> DSalliance = DriverStation.getAlliance();
            if (DSalliance.isPresent()) alliance = DSalliance.get();
            Log.info("Alliance", alliance.toString());
        }

        // if (RobotContainer.allianceRead.getAsBoolean()){
        //     alliance = RobotContainer.allianceWrite.getAsBoolean() ? Alliance.Red : Alliance.Blue;
        //     Log.info("Alliance", alliance.toString());
        // }

        return Alliance.Blue;
    }

    public static Robot instance;

    public static RobotContainer m_robotContainer;
    public static AutoPrograms autoPrograms;
    // public static AutoPrograms autoPrograms;

    public static synchronized Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }

    private Robot() {
        Log.profile("init RobotContainer", () -> m_robotContainer = new RobotContainer());
        autoPrograms = AutoPrograms.getInstance();
    }

    @Override
    public void robotInit(){
        Arm.getInstance().pivot.reset(180);
        Camera.enableAll();
        m_robotContainer.initDashboard();
        Log.info("Dashboard", "Done");
        LiveWindow.disableAllTelemetry();
        // Log.logDebug = true;
        autoPrograms.initAutoSelector();
        Log.Type.enable(STATE_MACHINE_PRIMARY, STATE_MACHINE_SECONDARY, MECHANISM, MOTOR);
        PathfindingCommand.warmupCommand().schedule();
        // Swerve.getInstance().resetGyro(0);
    }

    @Override
    public void driverStationConnected() {
        Log.info("State", "DS Connected");
        Log.info("Alliance", getAlliance().toString());
    }

    @Override
    public void robotPeriodic(){
        CommandScheduler.getInstance().run();
        Camera.updateAll();
    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();
        Camera.enableAll();
        runOnce(()-> {
            Swerve.translationController.disable();
            Swerve.rotationController.disable();
        }).schedule();
        
        Command m_autonomousCommand = autoPrograms.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        else System.out.println("Auto Command is null");
    }

    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousExit() {
        CommandScheduler.getInstance().cancelAll();
        Commands.runOnce(()-> Swerve.autoMoveEnabled = false).schedule();
    }

    @Override
    public void simulationInit() {
        for (FieldStates f : FieldStates.values()) {
            System.out.println(f.name() + ": " + f.getPose2d());
            System.out.println(f.name() + " Back: " + f.getBackPose2d());
        }
    }

    List<PositionSubsystemBase> pidSubsystems = List.of(Arm.getInstance().pivot, Elevator.getInstance().elevator, Intake.getInstance().pivot);

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
        Camera.updateAll();
    }

    // @Override
    // public void simulationInit() {
        
    // }

    // @Override
    // public void simulationPeriodic() {
    //     CommandScheduler.getInstance().run();
    // }

    @Override
    public void teleopExit() {
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        Swerve.disable();
        Arm.getInstance().overrideState(ArmStates.START);
        Climber.getInstance().overrideState(ClimberStates.START);
        Elevator.getInstance().overrideState(ElevatorStates.START);
        Intake.getInstance().overrideState(IntakeStates.START);
        Superstructure.getInstance().overrideState(SuperstructureStates.START);
    }

    @Override
    public void disabledExit() {
        for (var subsystem : pidSubsystems) {
            subsystem.run(0);
        }
        for (var subsystem : pidSubsystems) {
            subsystem.startPID(subsystem.getPosition());
        }
    }
    
    // @Override
    // public void disabledPeriodic() {
    //     CommandScheduler.getInstance().run();
    // }
}
