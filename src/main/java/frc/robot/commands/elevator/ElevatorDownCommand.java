package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/** Moves the elevator down. When it ends, the elevator stops. */
public class ElevatorDownCommand extends Command {
    private final ElevatorSubsystem elevSubsystem;

    public ElevatorDownCommand(ElevatorSubsystem elevSubsystem) {
        this.elevSubsystem = elevSubsystem;
        addRequirements(elevSubsystem);
    }

    @Override
    public void execute() {
        elevSubsystem.elevateDown();
    }

    @Override
    public void end(boolean interrupted) {
        elevSubsystem.stop();
    }
}
