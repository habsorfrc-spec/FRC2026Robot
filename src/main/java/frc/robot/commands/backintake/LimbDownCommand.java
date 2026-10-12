package frc.robot.commands.backintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BackIntakeSubsystem;

/** Moves the back intake limb down. When it ends, the back intake motors stop. */
public class LimbDownCommand extends Command {
    private final BackIntakeSubsystem backSubsystem;

    public LimbDownCommand(BackIntakeSubsystem backSubsystem) {
        this.backSubsystem = backSubsystem;
        addRequirements(backSubsystem);
    }

    @Override
    public void execute() {
        backSubsystem.limbDown();
    }

    @Override
    public void end(boolean interrupted) {
        backSubsystem.stop();
    }
}
