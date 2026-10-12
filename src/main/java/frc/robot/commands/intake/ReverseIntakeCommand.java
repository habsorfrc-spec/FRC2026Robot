package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelSubsystem;

/** Runs the intake in reverse. When it ends, all the fuel motors stop. */
public class ReverseIntakeCommand extends Command {
    private final FuelSubsystem ballSubsystem;

    public ReverseIntakeCommand(FuelSubsystem ballSubsystem) {
        this.ballSubsystem = ballSubsystem;
        addRequirements(ballSubsystem);
    }

    @Override
    public void execute() {
        ballSubsystem.revIntake();
    }

    @Override
    public void end(boolean interrupted) {
        ballSubsystem.stop();
    }
}
