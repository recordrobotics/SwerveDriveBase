package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
// WPILib imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotState.Mode;
// Local imports
import frc.robot.commands.KillSpecified;
import frc.robot.control.*;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.*;
import frc.robot.utils.ModuleConstants.InvalidConfigException;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdRoutine;
import frc.robot.utils.libraries.Elastic;
import frc.robot.utils.libraries.Elastic.Notification;
import frc.robot.utils.libraries.Elastic.Notification.NotificationLevel;

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
    public static final int CONTROL_XBOX_PORT = 0;

    public static final double INVALID_COMMAND_VIBRATE_TIME = 0.1;

    public static Drivetrain drivetrain;
    public static PoseSensorFusion poseSensorFusion;
    public static Lights lights;
    public static PowerDistributionPanel pdp;
    public static RobotModel model;

    private Alert noEncoderResetAlert = new Alert("Encoders not reset!", AlertType.kError);

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
        lights = new Lights();
        pdp = new PowerDistributionPanel();
        model = new RobotModel();

        // Sets up Control scheme chooser
        DashboardUI.Overview.addControls(new OnlyJoystick(CONTROL_JOYSTICK_PORT));

        configureTriggers();

        noEncoderResetAlert.set(true);

        if (Constants.RobotState.getMode() != Mode.REAL) {
            // No point in manually resetting encoders in simulation since starting config is always in the right spot
            resetEncoders();
        }
    }

    public void teleopInit() {
        /* nothing to do */
    }

    public void disabledInit() {
        /* nothing to do */
    }

    public void disabledExit() {
        // Reset encoders when enabling if not already reset (safety measure)
        if (noEncoderResetAlert.get()) {
            resetEncoders();
        }
    }

    private void configureTriggers() {

        // Command to kill robot
        new Trigger(() -> DashboardUI.Overview.getControl().isKillTriggered())
                .whileTrue(
                        new KillSpecified(drivetrain).alongWith(new InstantCommand(() -> CommandScheduler.getInstance()
                                .cancelAll())));

        // Reset pose trigger
        new Trigger(() -> DashboardUI.Overview.getControl().isPoseResetTriggered())
                .onTrue(new InstantCommand(poseSensorFusion::resetDriverPose));
        new Trigger(DashboardUI.Overview::isResetLocationPressed)
                .onTrue(new InstantCommand(poseSensorFusion::resetStartingPose).ignoringDisable(true));
        new Trigger(DashboardUI.Overview::isEncoderResetPressed)
                .onTrue(new InstantCommand(this::resetEncoders).ignoringDisable(true));
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

    public void simulationPeriodic() {
        updateSimulationBattery(drivetrain);
    }

    @SuppressWarnings("java:S2325")
    public void updateSimulationBattery(PoweredSubsystem... subsystems) {
        double[] currents = new double[subsystems.length];
        for (int i = 0; i < subsystems.length; i++) {
            currents[i] = subsystems[i].getCurrentDrawAmps();
        }
    }

    public void resetEncoders() {
        noEncoderResetAlert.set(false);
        Elastic.sendNotification(
                new Notification(NotificationLevel.INFO, "Encoders reset!", "Successfully reset arm encoders."));
    }

    /** frees up all hardware allocations */
    public static void close() throws Exception {
        drivetrain.close();
        poseSensorFusion.close();
        pdp.close();
    }
}
