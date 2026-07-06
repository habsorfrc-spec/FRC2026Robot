/*package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.epilogue.logging.LogBackedSendableBuilder;
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
    Long start1, start2 =  System.currentTimeMillis();

    double relativePos = 0;
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
    drive.setSafetyEnabled(false);
    gyro.reset();
  }

  // =========================
  // Periodic Debug
  // =========================
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Yaw", gyro.getYaw());
    SmartDashboard.putNumber("Turn Error", turnPID.getPositionError());
    kp = SmartDashboard.getNumber("kp", kp);
    kd = SmartDashboard.getNumber("kd", kd);
    ki = SmartDashboard.getNumber("ki", ki);
    SmartDashboard.putNumber("Average distance", getRelativeDistance());
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
  public Command driveDistance(double meters, double speed, int time) {

    double direction = Math.signum(meters);

    return new FunctionalCommand(

        () -> {
          isStandOnPlace = false;
          start1 = System.currentTimeMillis();
          relativePos = getAverageDistance();
          resetEncoders();
        },

        () -> driveToY(meters,speed, time),//driveForward(Math.abs(speed) * direction),

        interrupted -> stop(),

        () -> {
          return isStandOnPlace;
        },

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

    public void driveToY(double distance, double maxSpeed, int time){
      if(maxSpeed > 1)
          maxSpeed = 1;
      else if(maxSpeed < 0)
          maxSpeed = 0.2;
      
      PIDController pidController = new PIDController(DriveConstants.kp, DriveConstants.ki, DriveConstants.kd);//new PIDController(DriveConstants.kp, DriveConstants.ki, DriveConstants.kd);
      if(!isOnPos(distance) && start1 - System.currentTimeMillis() < 2000) {
        driveForward(pidController.calculate(getRelativeDistance(), distance)*maxSpeed);
        start2 = System.currentTimeMillis();
      }
      else if (System.currentTimeMillis() - start2 < time) {
        driveForward(pidController.calculate(getRelativeDistance(), distance)*maxSpeed);
      }else{
        isStandOnPlace = true;
      }
    }

    public double getRelativeDistance(){
      return getAverageDistance() - relativePos;
    }

    public boolean isOnPos(double posY){
      double std = 0.4;
      return Math.abs(getRelativeDistance() - posY) <= std; 
    }

    public Command changeSpeed(){
      return new FunctionalCommand(
         () -> {
          if(DRIVE_SCALING == 1)
          {
            DRIVE_SCALING = 0.5;
            ROTATION_SCALING = 0.4;
          }
          else
          {
            DRIVE_SCALING = 1;
            ROTATION_SCALING = 0.8;
          }
         },
         
         () -> {},
         
         interrupted -> stop(), 
         
         () -> {
          return true;
        }
        );
    }
}*/
///////////////קוד של עידן למחוק עם הקוד עושה בעיות ////////////////////
package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode; // הייבוא המדויק ל-2025

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

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
    Long start1, start2 =  System.currentTimeMillis();

    double relativePos = 0;
    
    // משתנה מעקב פנימי למניעת הצפת ערוץ ה-CAN
    private boolean m_isBrakeMode = false; 

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
    leaderConfig.idleMode(IdleMode.kCoast); // מתחילים ב-Coast כברירת מחדל

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
    followerConfig.idleMode(IdleMode.kCoast);

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
    drive.setSafetyEnabled(false);
    gyro.reset();
  }

  // =========================
  // פונקציית עדכון חכמה שמשנה מצב *רק* כשיש מעבר אמיתי בין נסיעה לעצירה
  // =========================
  private void updateIdleMode(boolean desireBrake) {
      if (m_isBrakeMode != desireBrake) {
          m_isBrakeMode = desireBrake;
          SparkMaxConfig config = new SparkMaxConfig();
          config.idleMode(desireBrake ? IdleMode.kBrake : IdleMode.kCoast);
          
          leftLeader.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
          leftFollower.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
          rightLeader.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
          rightFollower.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      }
  }

  // =========================
  // Periodic Debug
  // =========================
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Yaw", gyro.getYaw());
    SmartDashboard.putNumber("Turn Error", turnPID.getPositionError());
    kp = SmartDashboard.getNumber("kp", kp);
    kd = SmartDashboard.getNumber("kd", kd);
    ki = SmartDashboard.getNumber("ki", ki);
    SmartDashboard.putNumber("Average distance", getRelativeDistance());
    SmartDashboard.putBoolean("Drivetrain Brake Mode Active", m_isBrakeMode);
  }

  // =========================
  // Drive Methods
  // =========================
  public void arcadeDrive(double xSpeed, double zRotation) {
    // אם הנהג לא מנסה לנסוע או להסתובב (ערכים קטנים מ-0.05 בגלל Deadband של השלט)
    if (Math.abs(xSpeed) < 0.05 && Math.abs(zRotation) < 0.05) {
        updateIdleMode(true); // הפעל ברייק מוד אוטומטי בעצירה
    } else {
        updateIdleMode(false); // שחרר לקוסט מוד בנסיעה
    }
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return run(() -> arcadeDrive(xSpeed.getAsDouble(),
        zRotation.getAsDouble()));
  }

  public void stop() {
    updateIdleMode(true); // כשמבקשים לעצור, נועלים מיד
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
              double kS = 0.12;
              if (Math.abs(output) > 0.02) {
                output += Math.copySign(kS, output);
              }
              output = Math.max(Math.min(output, 0.6), -0.6);
              
              // עדכון מצב בלימה בזמן סיבוב אוטונומי
              if (Math.abs(output) < 0.05) {
                  updateIdleMode(true);
              } else {
                  updateIdleMode(false);
              }
              
              drive.tankDrive(output, -output);
            },
            this::stop
        ).until(turnPID::atSetpoint)
    );
  }

  public Command driveDistance(double meters, double speed, int time) {
    return new FunctionalCommand(
        () -> {
          isStandOnPlace = false;
          start1 = System.currentTimeMillis();
          relativePos = getAverageDistance();
          resetEncoders();
        },
        () -> driveToY(meters,speed, time),
        interrupted -> stop(),
        () -> {
          return isStandOnPlace;
        },
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
      // קורא למתודה הפנימית שלנו כדי לעבור דרך מנגנון ה-Brake/Coast האוטומטי
      this.arcadeDrive(speed, 0);
    }

    public void driveToY(double distance, double maxSpeed, int time){
      if(maxSpeed > 1)
          maxSpeed = 1;
      else if(maxSpeed < 0)
          maxSpeed = 0.2;
      
      PIDController pidController = new PIDController(DriveConstants.kp, DriveConstants.ki, DriveConstants.kd);
      if(!isOnPos(distance) && start1 - System.currentTimeMillis() < 2000) {
        driveForward(pidController.calculate(getRelativeDistance(), distance)*maxSpeed);
        start2 = System.currentTimeMillis();
      }
      else if (System.currentTimeMillis() - start2 < time) {
        driveForward(pidController.calculate(getRelativeDistance(), distance)*maxSpeed);
      }else{
        isStandOnPlace = true;
      }
    }

    public double getRelativeDistance(){
      return getAverageDistance() - relativePos;
    }

    public boolean isOnPos(double posY){
      double std = 0.4;
      return Math.abs(getRelativeDistance() - posY) <= std; 
    }

    public Command changeSpeed(){
      return new FunctionalCommand(
         () -> {
          if(DRIVE_SCALING == 1)
          {
            DRIVE_SCALING = 0.5;
            ROTATION_SCALING = 0.4;
          }
          else
          {
            DRIVE_SCALING = 1;
            ROTATION_SCALING = 0.8;
          }
         },
         () -> {},
         interrupted -> stop(), 
         () -> {
          return true;
        }
        );
    }
}