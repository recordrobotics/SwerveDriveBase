package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
// WPILib imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// Local imports
import frc.robot.control.*;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.*;
import frc.robot.utils.ModuleConstants.InvalidConfigException;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings({"java:S1444", "java:S1104", "java:S3010"})
public final class RobotContainer {

    public static final double ROBOT_PERIODIC = 0.02;
    public static final int CONTROL_JOYSTICK_PORT = 2;

    public static Drivetrain drivetrain;
    public static PoseSensorFusion poseSensorFusion;
    public static RobotModel model;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        try {
            drivetrain = new Drivetrain();
        } catch (InvalidConfigException e) {
            e.printStackTrace();
            DriverStation.reportError("Could not load drivetrain config!: " + e.getMessage(), false);
            return;
        }

        poseSensorFusion = new PoseSensorFusion();
        model = new RobotModel();

        // Sets up Control scheme chooser
        DashboardUI.Overview.addControls(new OnlyJoystick(CONTROL_JOYSTICK_PORT));

        configureTriggers();
    }

    public void teleopInit() {
        /* nothing to do */
    }

    public void disabledInit() {
        /* nothing to do */
    }

    private void configureTriggers() {
        // Reset pose trigger
        new Trigger(() -> DashboardUI.Overview.getControl().isPoseResetTriggered())
                .onTrue(new InstantCommand(poseSensorFusion::resetDriverPose));
        new Trigger(DashboardUI.Overview::isResetLocationPressed)
                .onTrue(new InstantCommand(poseSensorFusion::resetStartingPose).ignoringDisable(true));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        if (SysIdManager.getSysIdRoutine() != SysIdRoutine.NONE) {
            return SysIdManager.getSysIdRoutine().createCommand();
        }

        return Commands.none();
    }

    public void simulationPeriodic() {}

    /** frees up all hardware allocations */
    public static void close() throws Exception {
        drivetrain.close();
    }
}
