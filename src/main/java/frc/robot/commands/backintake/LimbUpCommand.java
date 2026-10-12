package frc.robot.commands.backintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BackIntakeSubsystem;

/** Moves the back intake limb up. When it ends, the back intake motors stop. */
public class LimbUpCommand extends Command {
    private final BackIntakeSubsystem backSubsystem;

    public LimbUpCommand(BackIntakeSubsystem backSubsystem) {
        this.backSubsystem = backSubsystem;
        addRequirements(backSubsystem);
    }

    @Override
    public void execute() {
        backSubsystem.limbUp();
    }

    @Override
    public void end(boolean interrupted) {
        backSubsystem.stop();
    }
}
