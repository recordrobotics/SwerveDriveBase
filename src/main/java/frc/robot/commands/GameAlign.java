package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.hybrid.AlignToPose;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.CommandUtils;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class GameAlign {

  public static Command alignTarget(
      Supplier<Pose2d> target,
      Transform2d pathTargetTransform,
      boolean usePath,
      boolean useAlign,
      boolean repeatedly,
      double pathTimeout,
      double alignTimeout) {
    Pose2d targetPose = target.get();

    Pose2d pathTarget = null;
    if (usePath) {
      pathTarget = targetPose.transformBy(pathTargetTransform);
    }

    Command alignCmd = null;
    if (useAlign) {
      alignCmd =
          CommandUtils.finishOnInterrupt(
              new AlignToPose(
                      () -> {
                        return target.get();
                      })
                  .withTimeout(alignTimeout));

      if (repeatedly) {
        alignCmd =
            alignCmd.andThen(
                CommandUtils.finishOnInterrupt(
                        new AlignToPose(
                                () -> {
                                  return target.get();
                                })
                            .withTimeout(alignTimeout))
                    .repeatedly()
                    .onlyWhile(
                        () -> {
                          if (RobotState.isAutonomous()) return true;

                          if (AlignToPose.getDrivetrainControl() == null) return true;

                          var driverVelocity =
                              DashboardUI.Overview.getControl()
                                  .getDrivetrainControl()
                                  .getDriverVelocity();
                          var targetVelocity =
                              AlignToPose.getDrivetrainControl().getTargetVelocity();

                          // If the driver is manually adjusting, stop autoalign
                          if (Math.abs(driverVelocity.getX() - targetVelocity.getX()) > 0.3) {
                            return false;
                          }

                          if (Math.abs(driverVelocity.getY() - targetVelocity.getY()) > 0.3) {
                            return false;
                          }

                          if (Math.abs(
                                  driverVelocity.getRotation().getRadians()
                                      - targetVelocity.getRotation().getRadians())
                              > Units.degreesToRadians(20)) {
                            return false;
                          }

                          return true;
                        }));
      }
    }

    return new SequentialCommandGroup(
        usePath
            ? CommandUtils.finishOnInterrupt(PathAlign.create(pathTarget).withTimeout(pathTimeout))
            : Commands.none(),
        useAlign ? alignCmd : Commands.none());
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
