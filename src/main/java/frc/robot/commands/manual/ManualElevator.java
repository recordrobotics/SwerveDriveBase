package frc.robot.commands.manual;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.control.AbstractControl;
import frc.robot.dashboard.DashboardUI;

public class ManualElevator extends Command {
  public ManualElevator() {
    addRequirements(RobotContainer.elevator);
  }

  private double height = 0;

  @Override
  public void initialize() {
    height = RobotContainer.elevator.getCurrentHeight();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AbstractControl controls = DashboardUI.Overview.getControl();

    LinearVelocity manualElevatorVelocity = controls.getManualElevatorVelocity();
    height += manualElevatorVelocity.times(Milliseconds.of(20)).in(Meters);
    height =
        MathUtil.clamp(height, Constants.Elevator.STARTING_HEIGHT, Constants.Elevator.MAX_HEIGHT);

    TrapezoidProfile.State goal =
        new TrapezoidProfile.State(height, manualElevatorVelocity.in(MetersPerSecond));

    if (goal.position
        < Constants.Elevator.STARTING_HEIGHT
            + Constants.Elevator.MANUAL_CONTROL_MARGIN.in(Meters)) {
      goal.velocity = Math.max(goal.velocity, 0);
    } else if (goal.position
        > Constants.Elevator.MAX_HEIGHT - Constants.Elevator.MANUAL_CONTROL_MARGIN.in(Meters)) {
      goal.velocity = Math.min(goal.velocity, 0);
    }

    RobotContainer.elevator.setGoal(goal);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
