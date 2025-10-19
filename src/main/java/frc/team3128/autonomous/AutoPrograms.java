package frc.team3128.autonomous;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.FlippingUtil.FieldSymmetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.Constants.FieldConstants.FieldStates;
import frc.team3128.Robot;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Superstructure.Superstructure;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import common.utility.Log;

import static frc.team3128.Constants.FieldConstants.allianceFlip;
import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.subsystems.Superstructure.SuperstructureStates.*;


/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang, Lucas Han
 */

public class AutoPrograms {

    private HashMap<String, Command> autoMap = new HashMap<String, Command>();
    private HashMap<String, Command> pathMap = new HashMap<String, Command>();
    private static Swerve swerve = Swerve.getInstance();
    private RobotConfig robotConfig;
    private static AutoPrograms instance;
    SendableChooser<Command> autoChooser;
    private Superstructure superstructure;

    private AutoPrograms() {
        superstructure = Superstructure.getInstance();
        configPathPlanner();
        initAutoSelector();
    }

    public static synchronized AutoPrograms getInstance() {
        if (instance == null) instance = new AutoPrograms();
        return instance;
    }


    public void initAutoSelector() {
        autoMap.clear();
        pathMap.clear();
        List<String> autoStrings = AutoBuilder.getAllAutoNames();

        for(String autoName: autoStrings) {
            autoMap.put(autoName, getPathPlannerAuto(autoName));
            try {
                for(PathPlannerPath path : PathPlannerAuto.getPathGroupFromAutoFile(autoName)) {
                    pathMap.put(path.name, AutoBuilder.followPath(path));
                }
            } catch(Exception e) {}
        }
        //autoMap.put("RB_3pc_EDC_auto", getPathPlannerAuto("RB_3pc_EDC_auto"));
        // NarwhalDashboard.getInstance().addAutos(autoStrings.toArray(new String[0]));
    }

    private void configPathPlanner() {
        Pathfinding.setPathfinder(new LocalADStar());

        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            robotConfig = new RobotConfig(
                ROBOT_MASS,
                ROBOT_MOI, 
                new ModuleConfig(
                    DRIVE_WHEEL_DIAMETER / 2, 
                    4.1, 
                    WHEEL_COF, 
                    DCMotor.getKrakenX60(1),
                    DRIVE_MOTOR_GEAR_RATIO, 
                    DRIVE_MOTOR_CURRENT_LIMIT, 
                    1
                ),
                Swerve.moduleOffsets
            );
        }

        FlippingUtil.symmetryType = FieldSymmetry.kRotational;
        AutoBuilder.configure(
            swerve::getPose, 
            swerve::resetOdometry, 
            swerve::getRobotVelocity, 
            (velocity, feedforwards)-> swerve.drive(ChassisSpeeds.fromRobotRelativeSpeeds(velocity, swerve.getGyroRotation2d())), 
            new PPHolonomicDriveController(
                new PIDConstants(Swerve.translationConfig.kP, Swerve.translationConfig.kI, Swerve.translationConfig.kD),
                new PIDConstants(Swerve.rotationConfig.kP, Swerve.rotationConfig.kI, Swerve.rotationConfig.kD)
            ),
            robotConfig,
            ()-> Robot.getAlliance() == Alliance.Red,
            swerve
        );
        NamedCommands.registerCommand(
            "Start",
            runOnce(() -> {
                superstructure.overrideState(HELD_NEUTRAL);
                swerve.resetGyro(Robot.getAlliance() == Alliance.Red ? 0 : 180);
            })
        );
        for (FieldStates state : FieldStates.values()) {
            if (state.name().length() == 1) {
                NamedCommands.registerCommand(
                    "Front L4 " + state.name(),
                    parallel(
                        superstructure.alignScoreCoralAuto(() -> allianceFlip(state.getPose2d())),
                        sequence(
                            waitSeconds(0.5),
                            superstructure.setStateCommand(PRE_L4)
                        )
                    ).withDeadline(
                        sequence(
                            waitSeconds(0.5),
                            waitUntil(() -> superstructure.stateEquals(HELD_NEUTRAL))
                        )
                    )
                );
                
                NamedCommands.registerCommand(
                    "Back L4 " + state.name(),
                    parallel(
                        superstructure.alignScoreCoralAuto(() -> allianceFlip(state.getBackPose2d())),
                        sequence(
                            waitSeconds(0.5),
                            superstructure.setStateCommand(PRE_L4)
                        )
                    ).withDeadline(
                        sequence(
                            waitSeconds(0.5),
                            waitUntil(() -> superstructure.stateEquals(HELD_NEUTRAL))
                        )
                    )
                );
            }
        }
    }

    public static Command getPathPlannerAuto(String trajectoryName) {
        return AutoBuilder.buildAuto(trajectoryName);
    }  

    public static Command getPathPlannerPath(String name) throws Exception {
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(name));
    }

    public Command getAuto(String name) {
        return autoMap.get(name);
    }

    public Command getPath(String name) {
        return pathMap.get(name);
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = null;
        // String selectedAutoName = NarwhalDashboard.getInstance().getSelectedAuto(); //NarwhalDashboard.getInstance().getSelectedAuto();
        // String hardcode = "MID_3pc_H_auto";
        // String hardcode = "LB_3pc_ILK_auto";
        // String hardcode = "MID_1pc_H_auto"; 
        // String hardcode = "Left_Leave_Backwards";
        String hardcode = "proc_1pc_b";
        
         
        Command autoCommand;
        if (selectedAutoName == null) {
            selectedAutoName = hardcode;
        }
        else if (selectedAutoName.equals("default")) {
            defaultAuto();
        }
        autoCommand = autoMap.get(selectedAutoName);

        Log.info("AUTO_SELECTED", selectedAutoName);
        return autoCommand;
    }

    private Command defaultAuto(){
        return none();
    }

    private Command reset() {
        return runOnce(()->swerve.resetGyro(Robot.getAlliance() == Alliance.Red ? 0 : 180));
    }

    public Command pathToPose(Pose2d targetPose) {
        PathConstraints constraints = new PathConstraints(
                3.0, 3.0,
                Units.degreesToRadians(171), Units.degreesToRadians(360));

        return AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0
        );
    }

    public Command pathToNearestPose(List<Pose2d> poses) {
        return pathToPose(swerve.getPose().nearest(poses));
    }
}
