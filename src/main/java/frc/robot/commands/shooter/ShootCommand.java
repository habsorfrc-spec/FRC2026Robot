package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.backintake.BackIntakeCommand;
import frc.robot.subsystems.BackIntakeSubsystem;
import frc.robot.subsystems.FuelSubsystem;

/** Shoots at RPM_CLOSE (set in FuelSubsystem) and runs the back intake, both at the same time. */
public class ShootCommand extends ParallelCommandGroup {
    public ShootCommand(FuelSubsystem ballSubsystem, BackIntakeSubsystem backSubsystem) {
        addCommands(
                ballSubsystem.launchCommand(),
                new BackIntakeCommand(backSubsystem));
    }
}
