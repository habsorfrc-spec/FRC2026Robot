package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

// import edu.wpi.first.epilogue.logging.LogBackedSendableBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OperatorConstants.DRIVE_SCALING;
import static frc.robot.Constants.OperatorConstants.ROTATION_SCALING;

public class CANDriveSubsystem extends SubsystemBase {

    double kp = SmartDashboard.getNumber("kp", 0),
            ki = SmartDashboard.getNumber("ki", 0),
            kd = SmartDashboard.getNumber("kd", 0);

    boolean isStandOnPlace = false;
    Long start1, start2 = System.currentTimeMillis();

    double relativePos = 0;

    // =========================
    // Motors
    // =========================
    private final SparkMax leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);

    private final SparkMax leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);

    private final SparkMax rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);

    private final SparkMax rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);

    private final DifferentialDrive drive = new DifferentialDrive(leftLeader, rightLeader);

    // =========================
    // Gyro (navX2 on MXP)
    // =========================
    // private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    // =========================
    // Turn PID
    // =========================
    private final PIDController turnPID = new PIDController(0.02, 0.0, 0.002); // Stronger P and D

    public CANDriveSubsystem() {

        /// Right leader
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.voltageCompensation(12);
        rightConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
        rightConfig.inverted(false);

        rightLeader.configure(
                rightConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Left leader
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.voltageCompensation(12);
        leftConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
        leftConfig.inverted(true);

        leftLeader.configure(
                leftConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // PID setup
        turnPID.enableContinuousInput(-180, 180);
        turnPID.setTolerance(2.0);
        drive.setSafetyEnabled(false);
        // gyro.reset();
    }

    // =========================
    // Drive Methods
    // =========================
    public void arcadeDrive(double xSpeed, double zRotation) {
        drive.arcadeDrive(
                xSpeed * DRIVE_SCALING,
                zRotation * ROTATION_SCALING);
    }

    public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
        return run(() -> arcadeDrive(xSpeed.getAsDouble(),
                zRotation.getAsDouble()));
    }

    public void driveForward(double speed) {
        // Drive straight forward/back using arcadeDrive (no rotation)
        drive.arcadeDrive(speed, 0);
    }

    public void stop() {
        drive.stopMotor();
    }

    // =========================
    // SPEED MODE (POV Left switches between fast and slow)
    // =========================
    public Command changeSpeed() {
        return new FunctionalCommand(
                () -> {
                    if (DRIVE_SCALING == 1) {
                        DRIVE_SCALING = 0.5;
                        ROTATION_SCALING = 0.4;
                    } else {
                        DRIVE_SCALING = 1;
                        ROTATION_SCALING = 0.8;
                    }
                },

                () -> {
                },

                interrupted -> stop(),

                () -> {
                    return true;
                });
    }

    // =========================
    // RELATIVE TURN METHOD
    // =========================
    // public Command turnRelative(double degrees) {
    //     gyro.reset();
    //     return runOnce(() -> {
    //         double target = gyro.getYaw() + degrees;
    //         turnPID.setSetpoint(target);
    //     })
    //             .andThen(
    //                     runEnd(
    //                             () -> {

    //                                 double output = turnPID.calculate(gyro.getYaw());

    //                                 // Static friction compensation
    //                                 double kS = 0.12;

    //                                 if (Math.abs(output) > 0.02) {
    //                                     output += Math.copySign(kS, output);
    //                                 }

    //                                 // Clamp
    //                                 output = Math.max(Math.min(output, 0.6), -0.6);

    //                                 drive.tankDrive(output, -output);

    //                             },
    //                             this::stop).until(turnPID::atSetpoint));
    // }

    // =========================
    // DRIVE DISTANCE (used by the autos)
    // =========================
    public Command driveDistance(double meters, double speed, int time) {

        double direction = Math.signum(meters);

        return new FunctionalCommand(

                () -> {
                    isStandOnPlace = false;
                    start1 = System.currentTimeMillis();
                    relativePos = getAverageDistance();
                    resetEncoders();
                },

                () -> driveToY(meters, speed, time), // driveForward(Math.abs(speed) * direction),

                interrupted -> stop(),

                () -> {
                    return isStandOnPlace;
                },

                this);
    }

    public void driveToY(double distance, double maxSpeed, int time) {
        if (maxSpeed > 1)
            maxSpeed = 1;
        else if (maxSpeed < 0)
            maxSpeed = 0.2;

        PIDController pidController = new PIDController(DriveConstants.kp, DriveConstants.ki, DriveConstants.kd);// new
                                                                                                                 // PIDController(DriveConstants.kp,
                                                                                                                 // DriveConstants.ki,
                                                                                                                 // DriveConstants.kd);
        if (!isOnPos(distance) && start1 - System.currentTimeMillis() < 2000) {
            driveForward(pidController.calculate(getRelativeDistance(), distance) * maxSpeed);
            start2 = System.currentTimeMillis();
        } else if (System.currentTimeMillis() - start2 < time) {
            driveForward(pidController.calculate(getRelativeDistance(), distance) * maxSpeed);
        } else {
            isStandOnPlace = true;
        }
    }

    public boolean isOnPos(double posY) {
        double std = 0.4;
        return Math.abs(getRelativeDistance() - posY) <= std;
    }

    // =========================
    // Encoder Methods
    // =========================
    public void resetEncoders() {
        leftFollower.getEncoder().setPosition(0);
        rightLeader.getEncoder().setPosition(0);
    }

    public double getAverageDistance() {
        return (leftFollower.getEncoder().getPosition()
                + rightLeader.getEncoder().getPosition()) / 2.0;
    }

    public double getRelativeDistance() {
        return getAverageDistance() - relativePos;
    }

    // =========================
    // Periodic Debug
    // =========================
    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Yaw", gyro.getYaw());
        SmartDashboard.putNumber("Turn Error", turnPID.getPositionError());
        kp = SmartDashboard.getNumber("kp", kp);
        kd = SmartDashboard.getNumber("kd", kd);
        ki = SmartDashboard.getNumber("ki", ki);
        SmartDashboard.putNumber("Average distance", getRelativeDistance());
        
    }
}
