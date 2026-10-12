package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelSubsystem;

/**
 * Feeds the balls up toward the shooter.
 * When it ends, only the feeder and intake stop, so the shooter keeps spinning.
 */
public class FeedUpCommand extends Command {
    private final FuelSubsystem ballSubsystem;

    public FeedUpCommand(FuelSubsystem ballSubsystem) {
        this.ballSubsystem = ballSubsystem;
        addRequirements(ballSubsystem);
    }

    @Override
    public void execute() {
        ballSubsystem.feedUp();
    }

    @Override
    public void end(boolean interrupted) {
        ballSubsystem.stopIntake();
    }
}
