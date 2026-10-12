// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.backintake.LimbDownCommand;
import frc.robot.commands.backintake.LimbUpCommand;
import frc.robot.commands.elevator.ElevatorDownCommand;
import frc.robot.commands.elevator.ElevatorUpCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.subsystems.BackIntakeSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FuelSubsystem;

/**
 * All the autonomous routines.
 * RobotContainer puts them in the "Auto Mode" chooser on the dashboard.
 * They are built from the command files in the commands/ subfolders.
 */
public final class Autos {
    // Subsystems (the same ones RobotContainer creates)
    private final CANDriveSubsystem driveSubsystem;
    private final FuelSubsystem ballSubsystem;
    private final BackIntakeSubsystem backSubsystem;
    private final ElevatorSubsystem elevSubsystem;

    public Autos(CANDriveSubsystem driveSubsystem, FuelSubsystem ballSubsystem,
            BackIntakeSubsystem backSubsystem, ElevatorSubsystem elevSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.ballSubsystem = ballSubsystem;
        this.backSubsystem = backSubsystem;
        this.elevSubsystem = elevSubsystem;
    }

    // =========================
    // FULL AUTO SEQUENCES (the options in the "Auto Mode" chooser)
    // =========================

    // "Test Drive Auto" (the default option)
    public Command getTestDriveAuto() {
        return getShootAuto()
                .andThen(new LimbUpCommand(backSubsystem).withTimeout(3))
                .andThen(getClimbAuto());
    }

    // "Shoot Auto"
    public Command getShootAuto() {
        return driveSubsystem.driveDistance(-2.3, 1, 1000)
                .raceWith(new WaitCommand(3))
                .andThen(new LimbDownCommand(backSubsystem).withTimeout(1))
                .andThen(new ShootCommand(ballSubsystem, backSubsystem).withTimeout(5));
    }

    // "Climb Auto"
    public Command getClimbAuto() {
        return new ElevatorDownCommand(elevSubsystem).withTimeout(0.1)
                .andThen(new LimbDownCommand(backSubsystem).withTimeout(1.3))
                .andThen(new LimbUpCommand(backSubsystem).withTimeout(1.8))
                .andThen(new ElevatorUpCommand(elevSubsystem).withTimeout(0.7))
                .andThen(driveSubsystem.driveDistance(5.05, 0.65, 1000)
                        .raceWith(new WaitCommand(3)))
                .andThen(new ElevatorDownCommand(elevSubsystem).withTimeout(1.5));
    }

    // "Shoot & Climb"
    public Command getCombinedCommand() {
        return getShootAuto().andThen(driveSubsystem.turnRelative(180))
                .andThen(new ElevatorUpCommand(elevSubsystem).withTimeout(0.7))
                .andThen(driveSubsystem.driveDistance(7.4, 0.4, 1000)
                        .raceWith(new WaitCommand(3)))
                .andThen(new ElevatorDownCommand(elevSubsystem).withTimeout(1.5));
    }

    // =========================
    // NOT USED RIGHT NOW (kept for building new autos)
    // =========================

    public Command getDriveForward1() {
        return driveSubsystem.driveDistance(12.0, 0.6, 3000);
    }

    public Command getDriveBack1() {
        return driveSubsystem.driveDistance(-2.0, 0.6, 1000);
    }

    public Command getSpin1() {
        return driveSubsystem.turnRelative(180);
    }

    // Example autonomous command which drives forward for 1 second.
    public static final Command exampleAuto(CANDriveSubsystem driveSubsystem, FuelSubsystem ballSubsystem) {
        return new SequentialCommandGroup(
                // Drive backwards for .25 seconds. The driveArcadeAuto command factory
                // creates a command which does not end which allows us to control
                // the timing using the withTimeout decorator
                driveSubsystem.driveArcade(() -> 0.5, () -> 0).withTimeout(.25),
                // Stop driving. This line uses the regular driveArcade command factory so it
                // ends immediately after commanding the motors to stop
                driveSubsystem.driveArcade(() -> 0, () -> 0),
                // Spin up the launcher for 1 second and then launch balls for 9 seconds, for a
                // total of 10 seconds
                ballSubsystem.spinUpCommand().withTimeout(1),
                ballSubsystem.launchCommand().withTimeout(9),
                // Stop running the launcher
                ballSubsystem.runOnce(() -> ballSubsystem.stop()));
    }
}
