package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.backintake.BackIntakeCommand;
import frc.robot.subsystems.BackIntakeSubsystem;
import frc.robot.subsystems.FuelSubsystem;

/** Delivery shot: shoots at RPM_DELIEVERY (set in FuelSubsystem) and runs the back intake, both at the same time. */
public class DeliveryShotCommand extends ParallelCommandGroup {
    public DeliveryShotCommand(FuelSubsystem ballSubsystem, BackIntakeSubsystem backSubsystem) {
        addCommands(
                ballSubsystem.launchDelieveryCommand(),
                new BackIntakeCommand(backSubsystem));
    }
}
