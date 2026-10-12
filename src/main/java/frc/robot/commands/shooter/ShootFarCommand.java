package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.backintake.BackIntakeCommand;
import frc.robot.subsystems.BackIntakeSubsystem;
import frc.robot.subsystems.FuelSubsystem;

/** Far shot: shoots at RPM_FAR (set in FuelSubsystem) and runs the back intake, both at the same time. */
public class ShootFarCommand extends ParallelCommandGroup {
    public ShootFarCommand(FuelSubsystem ballSubsystem, BackIntakeSubsystem backSubsystem) {
        addCommands(
                ballSubsystem.launchFarCommand(),
                new BackIntakeCommand(backSubsystem));
    }
}
