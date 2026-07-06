/*package frc.robot.subsystems;

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
        private final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);

        private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);

        // Autonomous chooser
        private final SendableChooser<Command> autoChooser = new SendableChooser<>();

        
    public RobotContainer() {
        
        configureBindings();

        // Build the Chooser
        // (These call the methods below, generating fresh commands safely)
        autoChooser.setDefaultOption("Test Drive Auto", getShootAuto().andThen(getUp().withTimeout(3)).andThen(getClimbAuto()));
        autoChooser.addOption("Shoot Auto", getShootAuto());
        autoChooser.addOption("Climb Auto", getClimbAuto());

        // Show chooser on dashboard
        SmartDashboard.putData("Auto Mode", autoChooser);

    } 

    public Command getDriveForward1() {
        return driveSubsystem.driveDistance(12.0, 0.6, 3000);
    }

    public Command getDriveBack1() {
        return driveSubsystem.driveDistance(-2.0, 0.6, 1000);
    }

    public Command getSpin1() {
        return driveSubsystem.turnRelative(180);
    }

    public Command getDown() {
        return backSubsystem.runEnd(
            () -> backSubsystem.limbDown(),
            () -> backSubsystem.stop()
        );
    }

    public Command getUp() {
        return backSubsystem.runEnd(
            () -> backSubsystem.limbUp(),
            () -> backSubsystem.stop()
        );
    }

    // =========================
    // 3. FULL AUTO SEQUENCES
    // =========================

    public Command getShootAuto() {
        return driveSubsystem.driveDistance(-2.3, 1, 1000)
            .raceWith(new WaitCommand(3))
            .andThen(getDown().withTimeout(1))
            .andThen(getShootCommand().withTimeout(5));
    }

    public Command getClimbAuto() {
        return 
            getDown().withTimeout(1).andThen(
        getUp().withTimeout(1.8))
            .andThen(getUpCommand().withTimeout(0.7))
            .andThen(driveSubsystem.driveDistance(9.2, 0.6, 1000)
                .raceWith(new WaitCommand(3)))
            //.andThen(driveSubsystem.driveDistance(1, 0.15, 500))
            //.andThen(getUpCommand().withTimeout(2))
            .andThen(getDownCommand().withTimeout(1.5));//);
    }

        private void configureBindings() {

                // Intake
                operatorController.leftBumper()
                                .whileTrue(
                                                ballSubsystem.runEnd(
                                                                () -> ballSubsystem.intake(),
                                                                () -> ballSubsystem.stop()));

                // Reverse intake
                operatorController.rightBumper()
                                .whileTrue(
                                                ballSubsystem.runEnd(
                                                                () -> ballSubsystem.revIntake(),
                                                                () -> ballSubsystem.stop()));

                // Elevator up
                operatorController.rightTrigger(0.1)
                                .whileTrue(
                                                elevSubsystem.runEnd(
                                                                () -> elevSubsystem.elevateUp(),
                                                                () -> elevSubsystem.stop()));

                // Elevator down
                operatorController.leftTrigger(0.1)
                                .whileTrue(
                                                elevSubsystem.runEnd(
                                                                () -> elevSubsystem.elevateDown(-5),
                                                                () -> elevSubsystem.stop()));

                // Eject
                operatorController.a()
                                .whileTrue(getShootCommand());



                // Back intake
                operatorController.b()
                                .whileTrue(
                                                backSubsystem.runEnd(
                                                                () -> backSubsystem.BackIntake(),
                                                                () -> backSubsystem.stop()));

                // Reverse back intake + intake
                operatorController.x()
                                .whileTrue(
                                                backSubsystem.runEnd(
                                                                () -> backSubsystem.revBackIntake(),
                                                                () -> backSubsystem.stop()).alongWith(
                                                                                ballSubsystem.runEnd(
                                                                                                () -> ballSubsystem
                                                                                                                .intake(),
                                                                                                () -> ballSubsystem
                                                                                                                .stop())));

                // Limb up
                operatorController.povUp()
                                .whileTrue(
                                                backSubsystem.runEnd(
                                                                () -> backSubsystem.limbUp(),
                                                                () -> backSubsystem.stop()));

                // Limb down
                operatorController.povDown()
                            .whileTrue(
                                            backSubsystem.runEnd(
                                                            () -> backSubsystem.limbDown(),
                                                            () -> backSubsystem.stop()));

                // Drive default command
                driveSubsystem.setDefaultCommand(
                                driveSubsystem.driveArcade(
                                                () -> -operatorController.getLeftY()
                                                                * DRIVE_SCALING,
                                                () -> -operatorController.getRightX()
                                                                * ROTATION_SCALING));

        }

        
         // =========================
         // RETURN AUTO TO ROBOT
         // =========================
         

        public Command getAutonomousCommand() {

                return autoChooser.getSelected();

        }

        public Command getUpCommand() {
                return elevSubsystem.runEnd(
                                () -> elevSubsystem.elevateUp(),
                                () -> elevSubsystem.stop());
        }

        public Command getDownCommand() {
                return elevSubsystem.runEnd(
                                () -> elevSubsystem.elevateDown(0),
                                () -> elevSubsystem.stop());
        }

        public Command getShootCommand() {
                return ballSubsystem.runEnd(
                                () -> ballSubsystem.eject(),
                                () -> ballSubsystem.stop()).alongWith(
                                                backSubsystem.runEnd(
                                                                () -> backSubsystem
                                                                                .BackIntake(),
                                                                () -> backSubsystem
                                                                                .stop()));
        }

}*/

/////////////////// קוד חדש של עידן וזיגר למחוק במיקרה של בעיה //////////////////////
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
        private boolean isStoped = true;
        // Subsystems
        private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
        private final FuelSubsystem ballSubsystem = new FuelSubsystem();
        private final BackIntakeSubsystem backSubsystem = new BackIntakeSubsystem();
        private final ElevatorSubsystem elevSubsystem = new ElevatorSubsystem();

        // Controllers
        private final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
        private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);

        // Autonomous chooser
        private final SendableChooser<Command> autoChooser = new SendableChooser<>();

        
    public RobotContainer() {
        
        configureBindings();

        // Build the Chooser
        autoChooser.setDefaultOption("Test Drive Auto", getShootAuto().andThen(getUp().withTimeout(3)).andThen(getClimbAuto()));
        autoChooser.addOption("Shoot Auto", getShootAuto());
        autoChooser.addOption("Climb Auto", getClimbAuto());
        autoChooser.addOption("Shoot & Climb", getCombinedCommand());

        // Show chooser on dashboard
        SmartDashboard.putData("Auto Mode", autoChooser);
    } 

    public Command getDriveForward1() {
        return driveSubsystem.driveDistance(12.0, 0.6, 3000);
    }

    public Command getDriveBack1() {
        return driveSubsystem.driveDistance(-2.0, 0.6, 1000);
    }

    public Command getSpin1() {
        return driveSubsystem.turnRelative(180);
    }

    public Command getDown() {
        return backSubsystem.runEnd(
            () -> backSubsystem.limbDown(),
            () -> backSubsystem.stop()
        );
    }

    public Command getUp() {
        return backSubsystem.runEnd(
            () -> backSubsystem.limbUp(),
            () -> backSubsystem.stop()
        );
    }

    // =========================
    // 3. FULL AUTO SEQUENCES
    // =========================

    public Command getShootAuto() {
        return driveSubsystem.driveDistance(-2.3, 1, 1000)
            .raceWith(new WaitCommand(3))
            .andThen(getDown().withTimeout(1))
            .andThen(getShootCommand().withTimeout(5));
    }

    public Command getClimbAuto() {
        return 
            getDown().withTimeout(1).andThen(
        getUp().withTimeout(1.8))
            .andThen(getUpCommand().withTimeout(0.7))
            .andThen(driveSubsystem.driveDistance(9.4, 0.4, 1000)
                .raceWith(new WaitCommand(3)))
            .andThen(getDownCommand().withTimeout(1.5));
    }

    public Command getCombinedCommand(){
        return getShootAuto().andThen(driveSubsystem.turnRelative(180))
            .andThen(getUpCommand().withTimeout(0.7))
            .andThen(driveSubsystem.driveDistance(9.4, 0.4, 1000)
                .raceWith(new WaitCommand(3)))
            .andThen(getDownCommand().withTimeout(1.5));
    }

        private void configureBindings() {

                // Intake
                operatorController.b()
                                .whileTrue(
                                                ballSubsystem.runEnd(
                                                                () -> ballSubsystem.intake(),
                                                                () -> ballSubsystem.stop()));

                // Reverse intake
                operatorController.x()
                                .whileTrue(
                                                ballSubsystem.runEnd(
                                                                () -> ballSubsystem.revIntake(),
                                                                () -> ballSubsystem.stop()));

                // Elevator up
                operatorController.rightTrigger(0.1)
                                .whileTrue(
                                                elevSubsystem.runEnd(
                                                                () -> elevSubsystem.elevateUp(),
                                                                () -> elevSubsystem.stop()));

                // Elevator down
                operatorController.leftTrigger(0.1)
                                .whileTrue(
                                                elevSubsystem.runEnd(
                                                                () -> elevSubsystem.elevateDown(-5),
                                                                () -> elevSubsystem.stop()));

                // Eject (ירי קרוב - כפתור A)
                operatorController.a()
                                .whileTrue(backSubsystem.runEnd(
                                                () -> backSubsystem.BackIntake(),
                                                () -> backSubsystem.stop())
                                .alongWith(ballSubsystem.runEnd(
                                    () -> ballSubsystem.feedUp(),
                                    () -> ballSubsystem.stopIntake())));

                // Shoot Far (ירי רחוק - כפתור Y החדש!)
                operatorController.y()
                                .whileTrue(getShootFarCommand());

                operatorController.povRight().toggleOnTrue(getReadyToShoot());

                operatorController.povLeft()
                                .toggleOnTrue(ChangeSpeed());             

                // Back intake
                operatorController.rightBumper()
                                .whileTrue(
                                                backSubsystem.runEnd(
                                                                () -> backSubsystem.BackIntake(),
                                                                () -> backSubsystem.stop()));

                // Reverse back intake + intake
                operatorController.leftBumper()
                                .whileTrue(
                                                  backSubsystem.runEnd(
                                                                () -> backSubsystem.revBackIntake(),
                                                                () -> backSubsystem.stop()).alongWith(
                                                                                ballSubsystem.runEnd(
                                                                                                () -> ballSubsystem.intake(),
                                                                                                () -> ballSubsystem.stop())));

                // Limb up
                operatorController.povUp()
                                .whileTrue(
                                                backSubsystem.runEnd(
                                                                () -> backSubsystem.limbUp(),
                                                                () -> backSubsystem.stop()));

                // Limb down
                operatorController.povDown()
                            .whileTrue(
                                            backSubsystem.runEnd(
                                                            () -> backSubsystem.limbDown(),
                                                            () -> backSubsystem.stop()));

                // Drive default command
                driveSubsystem.setDefaultCommand(
                                driveSubsystem.driveArcade(
                                                () -> -operatorController.getLeftY()
                                                                * DRIVE_SCALING,
                                                () -> -operatorController.getRightX()
                                                                * ROTATION_SCALING));
        }

        /*
         * =========================
         * RETURN AUTO TO ROBOT
         * =========================
         */

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public Command getUpCommand() {
                return elevSubsystem.runEnd(
                                () -> elevSubsystem.elevateUp(),
                                () -> elevSubsystem.stop());
        }

        public Command getDownCommand() {
                return elevSubsystem.runEnd(
                                () -> elevSubsystem.elevateDown(0),
                                () -> elevSubsystem.stop());
        }

        // פקודת קומבו לירי קרוב (מפעילה מנוע יורה קרוב ב-3000 + מנוע אחורי)
        public Command getShootCommand() {
                return ballSubsystem.launchCommand().alongWith(
                                backSubsystem.runEnd(
                                                () -> backSubsystem.BackIntake(),
                                                () -> backSubsystem.stop()));
        }

        public Command getReadyToShoot(){
            if(isStoped){

                ballSubsystem.warmUpRoller();
            }
            else{
                ballSubsystem.stop();
            }
            isStoped = !isStoped;
            return new WaitCommand(0.01);
        }

        // פקודת קומבו לירי רחוק (מפעילה מנוע יורה רחוק ב-4000 + מנוע אחורי)
        public Command getShootFarCommand() {
                return ballSubsystem.launchFarCommand().alongWith(
                                backSubsystem.runEnd(
                                                () -> backSubsystem.BackIntake(),
                                                () -> backSubsystem.stop()));
        }

        public Command getDelieveryCommand() {
            return ballSubsystem.launchDelieveryCommand().alongWith(
                                backSubsystem.runEnd(
                                                () -> backSubsystem.BackIntake(),
                                                () -> backSubsystem.stop()));   
                                                
        }

        public Command ChangeSpeed(){
            return driveSubsystem.changeSpeed();
        }
}