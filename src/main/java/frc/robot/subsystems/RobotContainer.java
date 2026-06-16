package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.FuelConstants.*;

import frc.robot.commands.Autos;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.BackIntakeSubsystem;

public class RobotContainer {

// Subsystems
private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
private final FuelSubsystem ballSubsystem = new FuelSubsystem();
private final BackIntakeSubsystem backSubsystem = new BackIntakeSubsystem();
private final ElevatorSubsystem elevSubsystem = new ElevatorSubsystem();

// Controllers
private final CommandXboxController driverController =
        new CommandXboxController(DRIVER_CONTROLLER_PORT);

private final CommandXboxController operatorController =
        new CommandXboxController(OPERATOR_CONTROLLER_PORT);

// Autonomous chooser
private final SendableChooser<Command> autoChooser =
        new SendableChooser<>();

public RobotContainer() {

    configureBindings();

    /* =========================
       BUILD YOUR AUTO COMMAND
       ========================= */

    Command driveForward1 =
            driveSubsystem.driveDistance(2.0, 0.4);

    Command driveForward2 =
            driveSubsystem.driveDistance(2.0, 0.4);

    Command spin1 =
            driveSubsystem.turnRelative(180);

    Command spin2 =
            driveSubsystem.turnRelative(180);

    Command down =
    backSubsystem.runOnce(
        () -> backSubsystem.limbDown()
    ).andThen(
        new WaitCommand(2.0)
    );

    Command intake =
            backSubsystem.runEnd(
                    () -> backSubsystem.BackIntake(),
                    () -> backSubsystem.stop()
            ).withTimeout(4.0);

    Command shoot =
            ballSubsystem.runEnd(
                    () -> ballSubsystem.eject(),
                    () -> ballSubsystem.stop()
            ).withTimeout(3.0);

    // Drive while intaking
    Command driveAndIntake =
            driveForward1.alongWith(intake);

    // Full autonomous sequence
    Command myAuto =
            spin1
                    .andThen(down)
                    .andThen(driveAndIntake)
                    .andThen(spin2)
                    .andThen(driveForward2)
                    .andThen(shoot);

    /* =========================
       ADD AUTO TO CHOOSER
       ========================= */

    autoChooser.setDefaultOption(
            "My Auto",
            myAuto
    );

    autoChooser.addOption(
            "Example Auto",
            Autos.exampleAuto(
                    driveSubsystem,
                    ballSubsystem
            )
    );

    // Show chooser on dashboard
    SmartDashboard.putData(
            "Auto Mode",
            autoChooser
    );
}

private void configureBindings() {

    // Intake
    operatorController.leftBumper()
            .whileTrue(
                    ballSubsystem.runEnd(
                            () -> ballSubsystem.intake(),
                            () -> ballSubsystem.stop()
                    )
            );

    // Reverse intake
    operatorController.rightBumper()
            .whileTrue(
                    ballSubsystem.runEnd(
                            () -> ballSubsystem.revIntake(),
                            () -> ballSubsystem.stop()
                    )
            );

    // Elevator up
    operatorController.rightTrigger(0.1)
            .whileTrue(
                    elevSubsystem.runEnd(
                            () -> elevSubsystem.elevateUp(),
                            () -> elevSubsystem.stop()
                    )
            );

    // Elevator down
    operatorController.leftTrigger(0.1)
            .whileTrue(
                    elevSubsystem.runEnd(
                            () -> elevSubsystem.elevateDown(),
                            () -> elevSubsystem.stop()
                    )
            );

    // Eject
    operatorController.a()
            .whileTrue(
                    ballSubsystem.runEnd(
                            () -> ballSubsystem.eject(),
                            () -> ballSubsystem.stop()
                    )
            );

    // Back intake
    operatorController.b()
            .whileTrue(
                    backSubsystem.runEnd(
                            () -> backSubsystem.BackIntake(),
                            () -> backSubsystem.stop()
                    )
            );

    // Reverse back intake + intake
    operatorController.x()
            .whileTrue(
                    backSubsystem.runEnd(
                            () -> backSubsystem.revBackIntake(),
                            () -> backSubsystem.stop()
                    ).alongWith(
                            ballSubsystem.runEnd(
                                    () -> ballSubsystem.intake(),
                                    () -> ballSubsystem.stop()
                            )
                    )
            );

    // Limb up
    operatorController.povUp()
            .whileTrue(
                    new RunCommand(
                            () -> backSubsystem.limbUp(),
                            backSubsystem
                    ).finallyDo(
                            interrupted -> backSubsystem.stop()
                    )
            );

    // Limb down
    operatorController.povDown()
            .whileTrue(
                    new RunCommand(
                            () -> backSubsystem.limbDown(),
                            backSubsystem
                    ).finallyDo(
                            interrupted -> backSubsystem.stop()
                    )
            );

    // Drive default command
    driveSubsystem.setDefaultCommand(
            driveSubsystem.driveArcade(
                    () -> -operatorController.getLeftY()
                            * DRIVE_SCALING,
                    () -> -operatorController.getRightX()
                            * ROTATION_SCALING
            )
    );
}

/* =========================
   RETURN AUTO TO ROBOT
   ========================= */

public Command getAutonomousCommand() {

    return autoChooser.getSelected();

}

}
