package frc.robot.commands.backintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BackIntakeSubsystem;

/** Runs the back intake rollers. When it ends, the back intake motors stop. */
public class BackIntakeCommand extends Command {
    private final BackIntakeSubsystem backSubsystem;

    public BackIntakeCommand(BackIntakeSubsystem backSubsystem) {
        this.backSubsystem = backSubsystem;
        addRequirements(backSubsystem);
    }

    @Override
    public void execute() {
        backSubsystem.BackIntake();
    }

    @Override
    public void end(boolean interrupted) {
        backSubsystem.stop();
    }
}
