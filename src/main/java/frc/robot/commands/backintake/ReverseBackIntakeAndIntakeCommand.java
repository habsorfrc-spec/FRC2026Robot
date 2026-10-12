package frc.robot.commands.backintake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.BackIntakeSubsystem;
import frc.robot.subsystems.FuelSubsystem;

/** Runs the back intake in reverse and the intake, both at the same time. */
public class ReverseBackIntakeAndIntakeCommand extends ParallelCommandGroup {
    public ReverseBackIntakeAndIntakeCommand(FuelSubsystem ballSubsystem, BackIntakeSubsystem backSubsystem) {
        addCommands(
                new ReverseBackIntakeCommand(backSubsystem),
                new IntakeCommand(ballSubsystem));
    }
}
