// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.function.BooleanSupplier;

/**
 * A command that runs another command repeatedly, restarting it when it ends, until this command is
 * interrupted. Command instances that are passed to it cannot be added to any other groups, or
 * scheduled individually.
 *
 * <p>The rules for command compositions apply: command instances that are passed to it cannot be
 * added to any other composition or scheduled individually,and the composition requires all
 * subsystems its components require.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class RepeatConditionallyCommand extends Command {
  private final Command m_command;
  private boolean m_ended;
  private BooleanSupplier m_condition;
  private boolean m_runAtleastOnce;

  /**
   * Creates a new RepeatCommand. Will run another command repeatedly, restarting it whenever it
   * ends, until this command is interrupted.
   *
   * @param command the command to run repeatedly
   */
  @SuppressWarnings("this-escape")
  public RepeatConditionallyCommand(
      Command command, BooleanSupplier condition, boolean runAtleastOnce) {
    m_command = requireNonNullParam(command, "command", "RepeatConditionallyCommand");
    m_condition = condition;
    m_runAtleastOnce = runAtleastOnce;
    CommandScheduler.getInstance().registerComposedCommands(command);
    addRequirements(command.getRequirements());
    setName("RepeatConditionally(" + command.getName() + ")");
  }

  @Override
  public void initialize() {
    m_ended = false;
    if (m_condition.getAsBoolean() || m_runAtleastOnce) {
      try {
        m_command.initialize();
      } catch (Exception e) {
        e.printStackTrace();
      }
    } else {
      m_ended = true;
    }
  }

  @Override
  public void execute() {
    if (m_ended && m_condition.getAsBoolean()) {
      m_ended = false;
      try {
        m_command.initialize();
      } catch (Exception e) {
        e.printStackTrace();
        return;
      }
    }
    try {
      m_command.execute();
    } catch (Exception e) {
      e.printStackTrace();
      return;
    }
    try {
      if (m_command.isFinished()) {
        // restart command
        m_command.end(false);
        m_ended = true;
      }
    } catch (Exception e) {
      e.printStackTrace();
      return;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // Make sure we didn't already call end() (which would happen if the command finished in the
    // last call to our execute())
    if (!m_ended) {
      try {
        m_command.end(interrupted);
      } catch (Exception e) {
        e.printStackTrace();
      }
      m_ended = true;
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return m_command.runsWhenDisabled();
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return m_command.getInterruptionBehavior();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("command", m_command::getName, null);
  }
}
