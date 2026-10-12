package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/** Moves the elevator up. When it ends, the elevator stops. */
public class ElevatorUpCommand extends Command {
    private final ElevatorSubsystem elevSubsystem;

    public ElevatorUpCommand(ElevatorSubsystem elevSubsystem) {
        this.elevSubsystem = elevSubsystem;
        addRequirements(elevSubsystem);
    }

    @Override
    public void execute() {
        elevSubsystem.elevateUp();
    }

    @Override
    public void end(boolean interrupted) {
        elevSubsystem.stop();
    }
}
