package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriveConstants.*;

public class CANDriveSubsystem extends SubsystemBase {

  // =========================
  // Motors
  // =========================
  private final SparkMax leftLeader =
      new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);

  private final SparkMax leftFollower =
      new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);

  private final SparkMax rightLeader =
      new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);

  private final SparkMax rightFollower =
      new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);

  private final DifferentialDrive drive =
      new DifferentialDrive(leftLeader, rightLeader);

  // =========================
  // Gyro (navX2 on MXP)
  // =========================
  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  // =========================
  // Turn PID
  // =========================
  private final PIDController turnPID =
      new PIDController(0.02, 0.0, 0.002); // Stronger P and D

  public CANDriveSubsystem() {

    // Leader config
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig.voltageCompensation(12);
    leaderConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

    // Apply to RIGHT leader first
    rightLeader.configure(
        leaderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Invert LEFT side
    leaderConfig.inverted(true);

    leftLeader.configure(
        leaderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Followers
    SparkMaxConfig followerConfig = new SparkMaxConfig();

    followerConfig.follow(leftLeader);
    leftFollower.configure(
        followerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    followerConfig.follow(rightLeader);
    rightFollower.configure(
        followerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // PID setup
    turnPID.enableContinuousInput(-180, 180);
    turnPID.setTolerance(2.0);

    gyro.reset();
  }

  // =========================
  // Periodic Debug
  // =========================
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Yaw", gyro.getYaw());
    SmartDashboard.putNumber("Turn Error", turnPID.getPositionError());
  }

  // =========================
  // Drive Methods
  // =========================
  public void arcadeDrive(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return run(() -> arcadeDrive(xSpeed.getAsDouble(),
        zRotation.getAsDouble()));
  }

  public void stop() {
    drive.stopMotor();
  }

  

  // =========================
  // RELATIVE TURN METHOD
  // =========================
  public Command turnRelative(double degrees) {
    gyro.reset();
    return runOnce(() -> {
      double target = gyro.getYaw() + degrees;
      turnPID.setSetpoint(target);
    })
    .andThen(
        runEnd(
            () -> {

              double output = turnPID.calculate(gyro.getYaw());

              // Static friction compensation
              double kS = 0.12;

              if (Math.abs(output) > 0.02) {
                output += Math.copySign(kS, output);
              }

              // Clamp
              output = Math.max(Math.min(output, 0.6), -0.6);

              drive.tankDrive(output, -output);

            },
            this::stop
        ).until(turnPID::atSetpoint)
    );
  }
  public Command driveDistance(double meters, double speed) {

    double direction = Math.signum(meters);

    return new FunctionalCommand(

        this::resetEncoders,

        () -> driveForward(Math.abs(speed) * direction),

        interrupted -> stop(),

        () -> Math.abs(getAverageDistance()) >= Math.abs(meters),

        this
    );
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

public void driveForward(double speed) {
  // Drive straight forward/back using arcadeDrive (no rotation)
  drive.arcadeDrive(speed, 0);
}
}