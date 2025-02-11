package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
// WPILib imports
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.commands.CoralIntakeFromGround;
import frc.robot.commands.CoralIntakeFromSource;
import frc.robot.commands.ElevatorMoveThenCoralShoot;
// Local imports
import frc.robot.commands.KillSpecified;
import frc.robot.commands.auto.PlannedAuto;
import frc.robot.commands.manual.ManualSwerve;
import frc.robot.commands.simulation.PlaceRandomGroundCoral;
import frc.robot.control.*;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.*;
import frc.robot.subsystems.io.CoralIntakeSim;
import frc.robot.subsystems.io.CoralShooterSim;
import frc.robot.subsystems.io.ElevatorAlgaeSim;
import frc.robot.subsystems.io.ElevatorSim;
import frc.robot.subsystems.io.GroundAlgaeSim;
import frc.robot.utils.AutoPath;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import org.photonvision.PhotonCamera;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final RobotModel model = new RobotModel();

  // Subsystems
  // TODO: replace with check if in simulation mode once subsystems actually connected
  public static final Drivetrain drivetrain = new Drivetrain();
  public static final PoseTracker poseTracker = new PoseTracker();
  public static final Limelight limelight = new Limelight();
  public static final Elevator elevator = new Elevator(new ElevatorSim(Constants.Elevator.kDt));
  public static final CoralShooter coralShooter = new CoralShooter(new CoralShooterSim(0.02));
  public static final CoralIntake coralIntake = new CoralIntake(new CoralIntakeSim(0.02));
  public static final ElevatorAlgae elevatorAlgae = new ElevatorAlgae(new ElevatorAlgaeSim(0.02));
  public static final GroundAlgae groundAlgae = new GroundAlgae(new GroundAlgaeSim(0.02));
  public static final Lights lights = new Lights();
  public static final PowerDistributionPanel pdp = new PowerDistributionPanel();

  public static final PhotonCamera camera = new PhotonCamera("photonvision");

  // Autonomous
  @SuppressWarnings("unused")
  private final AutoPath autoPath;

  @SuppressWarnings("unused")
  private Command autoCommand;

  private final GenericHID simulationController;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Sets up auto path
    autoPath = new AutoPath();

    DashboardUI.Autonomous.setupAutoChooser();

    // Sets up Control scheme chooser
    DashboardUI.Overview.addControls(new JoystickXbox(2, 0), new JoystickXboxKeypad(2, 0, 3));

    if (Constants.RobotState.getMode() == Mode.SIM) {
      simulationController = new GenericHID(4);
    } else {
      simulationController = null;
    }

    // Bindings and Teleop
    configureButtonBindings();

    ShuffleboardPublisher.setup(poseTracker.nav, drivetrain, limelight);

    drivetrain.setDefaultCommand(new ManualSwerve());

    // camera exposure fix
    camera.setDriverMode(false);
    NetworkTableInstance.getDefault().flush();
    camera.setDriverMode(true);
    NetworkTableInstance.getDefault().flush();
  }

  public void teleopInit() {}

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Command to kill robot
    new Trigger(() -> DashboardUI.Overview.getControl().getKill())
        .whileTrue(new KillSpecified(drivetrain))
        .onTrue(
            new InstantCommand(
                () ->
                    CommandScheduler.getInstance()
                        .cancel(
                            CommandScheduler.getInstance()
                                .requiring(
                                    drivetrain)))); // kill any commands that require drivetrain
    // (used for killing hybrid commands)

    // Reset pose trigger
    new Trigger(() -> DashboardUI.Overview.getControl().getPoseReset())
        .onTrue(new InstantCommand(poseTracker::resetDriverPose));

    new Trigger(() -> DashboardUI.Overview.getControl().getCoralShootL1())
        .onTrue(new ElevatorMoveThenCoralShoot(ElevatorHeight.L1));
    new Trigger(() -> DashboardUI.Overview.getControl().getCoralShootL2())
        .onTrue(new ElevatorMoveThenCoralShoot(ElevatorHeight.L2));
    new Trigger(() -> DashboardUI.Overview.getControl().getCoralShootL3())
        .onTrue(new ElevatorMoveThenCoralShoot(ElevatorHeight.L3));
    new Trigger(() -> DashboardUI.Overview.getControl().getCoralShootL4())
        .onTrue(new ElevatorMoveThenCoralShoot(ElevatorHeight.L4));
    new Trigger(() -> DashboardUI.Overview.getControl().getCoralGroundIntake())
        .onTrue(new CoralIntakeFromGround());
    new Trigger(() -> DashboardUI.Overview.getControl().getCoralSourceIntake())
        .onTrue(new CoralIntakeFromSource());
    new Trigger(() -> DashboardUI.Overview.getControl().getIntakeAlgae());
    new Trigger(() -> DashboardUI.Overview.getControl().getIntakeAlgae());
    new Trigger(() -> DashboardUI.Overview.getControl().getReefAlgae());
    new Trigger(() -> DashboardUI.Overview.getControl().getReefAlgae());
    new Trigger(() -> DashboardUI.Overview.getControl().getScoreAlgae());
    new Trigger(() -> DashboardUI.Overview.getControl().getClimb());

    // Simulation control commands
    if (Constants.RobotState.getMode() == Mode.SIM) {
      new Trigger(() -> simulationController.getRawButton(1)).onTrue(new PlaceRandomGroundCoral());
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autoCommand == null) {
      autoCommand = new PlannedAuto();
    }
    return autoCommand;

    // return elevator
    //     .sysIdQuasistatic(Direction.kForward)
    //     .andThen(elevator.sysIdQuasistatic(Direction.kReverse))
    //     .andThen(elevator.sysIdDynamic(Direction.kForward))
    //     .andThen(elevator.sysIdDynamic(Direction.kReverse));
  }

  public void testPeriodic() {
    DashboardUI.Test.testPeriodic();
  }

  public void simulationPeriodic() {
    updateSimulationBattery(
        drivetrain, elevator, coralShooter, coralIntake, elevatorAlgae, groundAlgae);
  }

  public void updateSimulationBattery(PoweredSubsystem... subsystems) {
    double[] currents = new double[subsystems.length];
    for (int i = 0; i < subsystems.length; i++) {
      currents[i] = subsystems[i].getCurrentDrawAmps();
    }

    RoboRioSim.setVInVoltage(
        MathUtil.clamp(BatterySim.calculateDefaultBatteryLoadedVoltage(currents), 0, 13));
  }

  /** frees up all hardware allocations */
  public void close() throws Exception {
    drivetrain.close();
    limelight.close();
    elevator.close();
    coralShooter.close();
    coralIntake.close();
    elevatorAlgae.close();
    groundAlgae.close();
    pdp.close();
  }
}
