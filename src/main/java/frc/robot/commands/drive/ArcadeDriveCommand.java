package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

/** Drives with arcade controls: forward/back speed and turn speed. Never ends on its own. */
public class ArcadeDriveCommand extends Command {
    private final CANDriveSubsystem driveSubsystem;
    private final DoubleSupplier xSpeed;
    private final DoubleSupplier zRotation;

    public ArcadeDriveCommand(CANDriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
        this.driveSubsystem = driveSubsystem;
        this.xSpeed = xSpeed;
        this.zRotation = zRotation;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble());
    }

    // No end(): when another command takes over the drive (like an auto), that command controls the motors.
}
