package frc.robot.commands.backintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BackIntakeSubsystem;

/** Runs the back intake rollers in reverse. When it ends, the back intake motors stop. */
public class ReverseBackIntakeCommand extends Command {
    private final BackIntakeSubsystem backSubsystem;

    public ReverseBackIntakeCommand(BackIntakeSubsystem backSubsystem) {
        this.backSubsystem = backSubsystem;
        addRequirements(backSubsystem);
    }

    @Override
    public void execute() {
        backSubsystem.revBackIntake();
    }

    @Override
    public void end(boolean interrupted) {
        backSubsystem.stop();
    }
}
