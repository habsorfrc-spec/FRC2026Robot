package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static frc.robot.Constants.OperatorConstants.*;

import frc.robot.commands.Autos;
import frc.robot.commands.backintake.BackIntakeCommand;
import frc.robot.commands.backintake.LimbDownCommand;
import frc.robot.commands.backintake.LimbUpCommand;
import frc.robot.commands.backintake.ReverseBackIntakeAndIntakeCommand;
import frc.robot.commands.drive.ArcadeDriveCommand;
import frc.robot.commands.elevator.ElevatorDownCommand;
import frc.robot.commands.elevator.ElevatorUpCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.ReverseIntakeCommand;
import frc.robot.commands.shooter.CloseShotCommand;
import frc.robot.commands.shooter.DeliveryShotCommand;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.BackIntakeSubsystem;
import frc.robot.subsystems.Limelight;

// =========================
// BUTTON MAP - operator controller (port 1)
// =========================
//   Left stick up/down       drive forward / back
//   Right stick left/right   turn
//   POV Left                 slow mode on/off
//
//   B                        intake
//   X                        reverse intake
//
//   A                        close shot: feed up + back intake
//   Y                        delivery shot (6000 RPM) + back intake
//   POV Right                shooter warm-up on/off
//
//   Right bumper             elevator up
//   Left bumper              elevator down
//
//   Right trigger            back intake
//   Left trigger             reverse back intake + intake
//   POV Up                   limb up
//   POV Down                 limb down
//
// The driver controller (port 0) has nothing bound yet.
// The commands themselves are in the commands/ folder, one file per command.

public class RobotContainer {
    // Subsystems
    private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
    private final FuelSubsystem ballSubsystem = new FuelSubsystem();
    private final BackIntakeSubsystem backSubsystem = new BackIntakeSubsystem();
    private final ElevatorSubsystem elevSubsystem = new ElevatorSubsystem();
    private final Limelight limeSubsystems = new Limelight();

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);

    // Autonomous (the auto routines are in commands/Autos.java)
    private final Autos autos = new Autos(driveSubsystem, ballSubsystem, backSubsystem, elevSubsystem);
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();
        configureAutoChooser();
    }

    // =========================
    // BUTTON BINDINGS
    // =========================

    private void configureBindings() {
        configureIntakeBindings();
        configureElevatorBindings();
        configureShooterBindings();
        configureDriveBindings();
        configureBackIntakeBindings();
    }

    // B, X
    private void configureIntakeBindings() {
        // Intake
        operatorController.b().whileTrue(new IntakeCommand(ballSubsystem));

        // Reverse intake
        operatorController.x().whileTrue(new ReverseIntakeCommand(ballSubsystem));
    }

    // Right bumper, Left bumper
    private void configureElevatorBindings() {
        // Elevator up
        operatorController.rightBumper().whileTrue(new ElevatorUpCommand(elevSubsystem));

        // Elevator down
        operatorController.leftBumper().whileTrue(new ElevatorDownCommand(elevSubsystem));
    }

    // A, Y, POV Right
    private void configureShooterBindings() {
        // Eject (ירי קרוב - כפתור A)
        operatorController.a().whileTrue(new CloseShotCommand(ballSubsystem, backSubsystem));

        // Delivery shot, 6000 RPM (Y used to run ShootFarCommand - ירי רחוק)
        operatorController.y().whileTrue(new DeliveryShotCommand(ballSubsystem, backSubsystem));

        // Shooter warm-up on/off
        operatorController.povRight().toggleOnTrue(ballSubsystem.warmUpStopToggleCommand(3500));

        // Auto-aim while holding the left-stick button
        operatorController.leftStick().whileTrue(
            new RunCommand(
                () -> {
                    double forward = -operatorController.getLeftY();

                    double tx = LimelightHelpers.getTX("limelight");
                    double rotation = -tx * 0.05;

                    driveSubsystem.arcadeDrive(forward, rotation);
                },
                driveSubsystem
            )
        );

        
    }

    // Sticks, POV Left
    private void configureDriveBindings() {
        // Slow mode on/off
        operatorController.povLeft().toggleOnTrue(driveSubsystem.changeSpeed());

        // Drive default command
        driveSubsystem.setDefaultCommand(
                new ArcadeDriveCommand(
                        driveSubsystem,
                        () -> -operatorController.getLeftY() * DRIVE_SCALING,
                        () -> -operatorController.getRightX() * ROTATION_SCALING));
    }

    // Right trigger, Left trigger, POV Up, POV Down
    private void configureBackIntakeBindings() {
        // Back intake
        operatorController.rightTrigger().whileTrue(new BackIntakeCommand(backSubsystem));

        // Reverse back intake + intake
        operatorController.leftTrigger().whileTrue(new ReverseBackIntakeAndIntakeCommand(ballSubsystem, backSubsystem));

        // Limb up
        operatorController.povUp().whileTrue(new LimbUpCommand(backSubsystem));

        // Limb down
        operatorController.povDown().whileTrue(new LimbDownCommand(backSubsystem));
    }

    // =========================
    // AUTONOMOUS
    // =========================

    private void configureAutoChooser() {
        // Build the Chooser
        autoChooser.setDefaultOption("Test Drive Auto", autos.getTestDriveAuto());
        autoChooser.addOption("Shoot Auto", autos.getShootAuto());
        autoChooser.addOption("Climb Auto", autos.getClimbAuto());
        //autoChooser.addOption("Shoot & Climb", autos.getCombinedCommand());

        // Show chooser on dashboard
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    // Return the chosen auto to Robot.java
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
