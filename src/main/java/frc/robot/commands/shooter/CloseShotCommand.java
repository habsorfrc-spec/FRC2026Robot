package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.backintake.BackIntakeCommand;
import frc.robot.subsystems.BackIntakeSubsystem;
import frc.robot.subsystems.FuelSubsystem;

/**
 * Close shot: runs the back intake and feeds the balls up, both at the same time.
 * It doesn't spin up the shooter itself; the warm-up toggle does that.
 */
public class CloseShotCommand extends ParallelCommandGroup {
    public CloseShotCommand(FuelSubsystem ballSubsystem, BackIntakeSubsystem backSubsystem) {
        addCommands(
                new BackIntakeCommand(backSubsystem),
                new FeedUpCommand(ballSubsystem));
    }
}
