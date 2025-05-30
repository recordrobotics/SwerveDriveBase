package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.hybrid.AlignToPose;
import frc.robot.dashboard.DashboardUI;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class GameAlign {
  private static boolean wasInterrupted = false;
  private static Transform2d alignStartDriverVelocity = Transform2d.kZero;

  public static Command alignTarget(
      Supplier<Pose2d> target,
      Transform2d pathTargetTransform,
      boolean usePath,
      boolean useAlign,
      boolean repeatedly) {
    Pose2d targetPose = target.get();

    Pose2d pathTarget = null;
    if (usePath) {
      pathTarget = targetPose.transformBy(pathTargetTransform);
    }

    Command alignCmd = null;
    if (useAlign) {
      alignCmd =
          new AlignToPose(
              () -> {
                return target.get();
              });

      if (repeatedly) alignCmd = alignCmd.repeatedly();
    }

    return new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  wasInterrupted = false;
                  if (!RobotState.isAutonomous() && AlignToPose.getDrivetrainControl() != null) {
                    alignStartDriverVelocity =
                        DashboardUI.Overview.getControl()
                            .getDrivetrainControl()
                            .getDriverVelocity();
                  } else {
                    alignStartDriverVelocity = Transform2d.kZero;
                  }
                }),
            usePath ? PathAlign.create(pathTarget) : Commands.none(),
            useAlign ? alignCmd : Commands.none())
        .raceWith(
            new WaitUntilCommand(
                    () -> {
                      if (RobotState.isAutonomous()) return false;

                      if (AlignToPose.getDrivetrainControl() == null) return false;

                      var driverVelocity =
                          DashboardUI.Overview.getControl()
                              .getDrivetrainControl()
                              .getDriverVelocity();
                      var targetVelocity = AlignToPose.getDrivetrainControl().getTargetVelocity();

                      // If the driver is moving in the opposite direction of the target velocity,
                      // interrupt
                      if (Math.signum(driverVelocity.getX()) != 0
                          && Math.signum(driverVelocity.getX())
                              != Math.signum(targetVelocity.getX())
                          && Math.abs(driverVelocity.getX() - targetVelocity.getX()) > 1.0
                          && Math.abs(alignStartDriverVelocity.getX() - driverVelocity.getX())
                              > 0.8) {
                        return true;
                      }

                      if (Math.signum(driverVelocity.getY()) != 0
                          && Math.signum(driverVelocity.getY())
                              != Math.signum(targetVelocity.getY())
                          && Math.abs(driverVelocity.getY() - targetVelocity.getY()) > 1.0
                          && Math.abs(alignStartDriverVelocity.getY() - driverVelocity.getY())
                              > 0.8) {
                        return true;
                      }

                      // if (Math.signum(driverVelocity.getRotation().getRadians()) != 0
                      //     && Math.signum(driverVelocity.getRotation().getRadians())
                      //         != Math.signum(targetVelocity.getRotation().getRadians())
                      //     && Math.abs(
                      //             driverVelocity.getRotation().getRadians()
                      //                 - targetVelocity.getRotation().getRadians())
                      //         > Units.degreesToRadians(20)
                      //     && Math.abs(
                      //             alignStartDriverVelocity.getRotation().getRadians()
                      //                 - driverVelocity.getRotation().getRadians())
                      //         > 0.3) {
                      //   return true;
                      // }

                      return false;
                    })
                .andThen(() -> wasInterrupted = true));
  }

  public static boolean wasInterrupted() {
    return wasInterrupted;
  }

  public static Command makeAlignWithCommand(
      BiFunction<Boolean, Boolean, Command> alignCommandFactory,
      BooleanSupplier waitCondition,
      Supplier<Command> performCommandFactory,
      BooleanSupplier switchCondition) {
    return Commands.either(
        alignCommandFactory
            .apply(true, true)
            .alongWith(new WaitUntilCommand(waitCondition).andThen(performCommandFactory.get())),
        alignCommandFactory
            .apply(true, false)
            .andThen(performCommandFactory.get())
            .andThen(alignCommandFactory.apply(false, true)),
        switchCondition);
  }
}
