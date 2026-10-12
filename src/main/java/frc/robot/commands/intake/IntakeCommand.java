package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelSubsystem;

/** Runs the intake. When it ends, all the fuel motors stop. */
public class IntakeCommand extends Command {
    private final FuelSubsystem ballSubsystem;

    public IntakeCommand(FuelSubsystem ballSubsystem) {
        this.ballSubsystem = ballSubsystem;
        addRequirements(ballSubsystem);
    }

    @Override
    public void execute() {
        ballSubsystem.intake();
    }

    @Override
    public void end(boolean interrupted) {
        ballSubsystem.stop();
    }
}
