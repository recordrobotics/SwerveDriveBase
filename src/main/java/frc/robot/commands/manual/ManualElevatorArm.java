package frc.robot.commands.manual;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.control.AbstractControl;
import frc.robot.dashboard.DashboardUI;

public class ManualElevatorArm extends Command {
  public ManualElevatorArm() {
    addRequirements(RobotContainer.elevatorArm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AbstractControl controls = DashboardUI.Overview.getControl();

    AngularVelocity manualElevatorVelocity = controls.getManualElevatorArmVelocity();
    if (RobotContainer.elevatorArm.getArmAngle() < Constants.ElevatorArm.MIN_POS + 0.1
        && manualElevatorVelocity.in(RadiansPerSecond) < 0) {
      manualElevatorVelocity = RadiansPerSecond.of(0);
    } else if (RobotContainer.elevatorArm.getArmAngle() > Constants.ElevatorArm.MAX_POS - 0.1
        && manualElevatorVelocity.in(RadiansPerSecond) > 0) {
      manualElevatorVelocity = RadiansPerSecond.of(0);
    }

    RobotContainer.elevatorArm.setManualVelocity(manualElevatorVelocity);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
