/*package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FuelConstants.*;

public class FuelSubsystem extends SubsystemBase {

    private final SparkMax feederRoller;
    private final SparkMax intakeLauncherRoller;
    private final TalonFX shooterRoller;

    private final RelativeEncoder feederEncoder;
    private final RelativeEncoder intakeEncoder;

    private final SparkClosedLoopController feederPID;
    private final SparkClosedLoopController intakePID;

    private final VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0);

    private final edu.wpi.first.wpilibj.Timer pulseTimer = new edu.wpi.first.wpilibj.Timer();
    private boolean pulseCycleStarted = false;

    private double intakeSetpoint = 0;
    private double feederSetpoint = 0;
    private double shooterSetpoint = 0; // ← added to track TalonFX target

    private double lastKP = 0;
    private double lastKI = 0;
    private double lastKD = 0;
    private double lastKV = 0;

    private boolean reached_speed = false;

    public FuelSubsystem() {

        intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
        feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
        shooterRoller = new TalonFX(SHOOTER_ID);

        feederEncoder = feederRoller.getEncoder();
        intakeEncoder = intakeLauncherRoller.getEncoder();

        feederPID = feederRoller.getClosedLoopController();
        intakePID = intakeLauncherRoller.getClosedLoopController();

        configureMotors();
        SmartDashboard.putNumber("Shooter kP", 0.14);
        SmartDashboard.putNumber("Shooter kI", 0.0);
        SmartDashboard.putNumber("Shooter kD", 0.0);
        SmartDashboard.putNumber("Shooter kV", 0.122);

        SmartDashboard.putNumber("Target Feeder RPM", 3000);
        SmartDashboard.putNumber("Target Intake RPM", 3000);
        SmartDashboard.putNumber("Target Shooter RPM", 3000); // ← added shooter tuning
        SmartDashboard.putNumber("Spin-up Feeder RPM", 2000);

        SmartDashboard.putData("Launch", new InstantCommand(this::launch));
    }

    private void configureMotors() {

        // ================= FEEDER =================

        SparkMaxConfig feederConfig = new SparkMaxConfig();

        feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);

        feederConfig.closedLoop
                .p(0.0002)
                .i(0.0)
                .d(0.0)
                .velocityFF(0.00018)
                .outputRange(-1.0, 1.0);

        feederConfig.closedLoopRampRate(0.15);

        feederRoller.configure(
                feederConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // ================= INTAKE LAUNCHER =================

        SparkMaxConfig launcherConfig = new SparkMaxConfig();

        launcherConfig.inverted(true);
        launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);

        launcherConfig.closedLoop
                .p(0.00022)
                .i(0.0)
                .d(0.00001)
                .velocityFF(0.00019)
                .outputRange(-1.0, 1.0);

        launcherConfig.closedLoopRampRate(0.2);

        intakeLauncherRoller.configure(
                launcherConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // ================= TALON SHOOTER PID =================

        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        configureShooterPID();
    }

    private void configureShooterPID() {

        TalonFXConfiguration talonConfig = new TalonFXConfiguration();

        talonConfig.Slot0.kP = SmartDashboard.getNumber("Shooter kP", 0.14);
        talonConfig.Slot0.kI = SmartDashboard.getNumber("Shooter kI", 0.0);
        talonConfig.Slot0.kD = SmartDashboard.getNumber("Shooter kD", 0.0);
        talonConfig.Slot0.kV = SmartDashboard.getNumber("Shooter kV", 0.12);

        shooterRoller.getConfigurator().apply(talonConfig);
    }

    public void setPIDF() {
        // optional live tuning later
        // עידן היה פה
    }

    
     //Sets velocity PID targets for the feeder, intake launcher, and TalonFX
     // shooter.
     //shooterRPM mirrors intakeRPM — adjust if they differ.
     
    public void setTargetRPM(double feederRPM, double intakeRPM) {

        feederSetpoint = feederRPM;
        intakeSetpoint = intakeRPM;
        shooterSetpoint = intakeRPM;

        feederPID.setReference(
                feederRPM,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0);

        intakePID.setReference(
                intakeRPM,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0);

        // TalonFX expects rotations/sec, not RPM
        shooterRoller.setControl(
                shooterVelocityRequest.withVelocity(intakeRPM / 60.0));
    }

    public void intake() {
        feederRoller.setVoltage(-INTAKING_FEEDER_VOLTAGE);
        intakeLauncherRoller.setVoltage(-INTAKING_INTAKE_VOLTAGE);
    }

    public void revIntake() {
        feederRoller.setVoltage(INTAKING_FEEDER_VOLTAGE);
        intakeLauncherRoller.setVoltage(INTAKING_INTAKE_VOLTAGE);
    }

    public void eject() {
        double ejectRPM = SmartDashboard.getNumber("Target Shooter RPM", 3000);
        shooterSetpoint = ejectRPM;

        // 1. Keep the main TalonFX shooter running at target speed
        shooterRoller.setControl(shooterVelocityRequest.withVelocity(ejectRPM / 60.0));

        // 2. Check if the shooter is actually at speed (using the tighter 5% tolerance)
        if (launcherAtSpeed(ejectRPM)) {

            // Start the clock for this feeding pulse cycle
            if (!pulseCycleStarted) {
                pulseTimer.restart();
                pulseCycleStarted = true;
            }

            // 3. The Pulse Engine: 0.15s ON, 0.30s OFF (Total cycle = 0.45s)
            if (pulseTimer.get() < 0.15) {
                // PHASE 1: Drive the feeder and intake forward to launch a ball
                reached_speed = true;
                intakeSetpoint = ejectRPM;
                feederSetpoint = ejectRPM;

                intakePID.setReference(-ejectRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
                feederPID.setReference(-ejectRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
            } else if (pulseTimer.get() < 0.45) {
                // PHASE 2: Mandatory stop to force a physical separation between balls
                feederRoller.stopMotor();
                intakeLauncherRoller.stopMotor();
                feederSetpoint = 0;
                intakeSetpoint = 0;
            } else {
                // PHASE 3: Reset the timer to initiate the next ball's feed cycle
                pulseTimer.restart();
            }

        } else {
            // If the shooter drops below speed (e.g. down to 2200 RPM),
            // instantly kill the cycle and freeze the feeder rollers.
            pulseCycleStarted = false;
            pulseTimer.stop();

            feederSetpoint = 0;
            feederRoller.stopMotor();
            intakeLauncherRoller.stopMotor();
            intakeSetpoint = 0;
        }
    }

    public void launch() {

        double targetFeederRPM = -SmartDashboard.getNumber("Target Feeder RPM", 3000);
        double targetIntakeRPM = SmartDashboard.getNumber("Target Intake RPM", 3000);

        // setTargetRPM now also drives the TalonFX
        setTargetRPM(targetFeederRPM, targetIntakeRPM);
    }

    public void spinUp() {

        double spinUpFeederRPM = SmartDashboard.getNumber("Spin-up Feeder RPM", 2000);
        double targetIntakeRPM = -SmartDashboard.getNumber("Target Intake RPM", 4500);

        // setTargetRPM now also drives the TalonFX
        setTargetRPM(spinUpFeederRPM, targetIntakeRPM);
    }

    public boolean launcherAtSpeed(double targetRPM) {
        double currentRPM = shooterRoller.getVelocity().getValueAsDouble() * 60.0;
        double error = Math.abs(currentRPM - targetRPM);

        if (reached_speed) {
            // If it drops more than 5% (150 RPM), pause the feeder!
            if (error < targetRPM * 0.05) {
                return true;
            } else {
                reached_speed = false;
                return false;
            }
        }

        // Require it to be within 3% (90 RPM) before firing the very first ball
        if (error < targetRPM * 0.03) {
            reached_speed = true;
            return true;
        }

        return false;
    }

    public void stop() {
        feederRoller.stopMotor();
        intakeLauncherRoller.stopMotor();
        shooterRoller.stopMotor();

        feederSetpoint = 0;
        intakeSetpoint = 0;
        shooterSetpoint = 0;

        // Reset the pulse tracking
        pulseCycleStarted = false;
        pulseTimer.stop();
        pulseTimer.reset();
    }

    public Command spinUpCommand() {
        return this.run(this::spinUp);
    }

    public Command launchCommand() {
        return this.run(this::launch).finallyDo(this::stop);
    }

    @Override
    public void periodic() {

        TalonFXConfiguration talonConfig = new TalonFXConfiguration();

        double kP = SmartDashboard.getNumber("Shooter kP", 0.14);
        double kI = SmartDashboard.getNumber("Shooter kI", 0.0);
        double kD = SmartDashboard.getNumber("Shooter kD", 0.0);
        double kV = SmartDashboard.getNumber("Shooter kV", 0.12);

        if (kP != lastKP || kI != lastKI || kD != lastKD || kV != lastKV) {

            TalonFXConfiguration cfg = new TalonFXConfiguration();

            cfg.Slot0.kP = kP;
            cfg.Slot0.kI = kI;
            cfg.Slot0.kD = kD;
            cfg.Slot0.kV = kV;

            shooterRoller.getConfigurator().apply(cfg);

            lastKP = kP;
            lastKI = kI;
            lastKD = kD;
            lastKV = kV;
        }

        SmartDashboard.putNumber("Feeder Velocity", feederEncoder.getVelocity());
        SmartDashboard.putNumber("Launcher Velocity", intakeEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter Roller RPM", shooterRoller.getVelocity().getValueAsDouble() * 60.0);

        SmartDashboard.putNumber("Launcher Setpoint", intakeSetpoint);
        SmartDashboard.putNumber("Shooter Setpoint", shooterSetpoint);

        SmartDashboard.putNumber("Launcher Error",
                intakeSetpoint - intakeEncoder.getVelocity());

        SmartDashboard.putNumber("Shooter Error",
                shooterSetpoint - shooterRoller.getVelocity().getValueAsDouble() * 60.0);

        SmartDashboard.putBoolean("Launcher Ready", launcherAtSpeed(shooterSetpoint));

        SmartDashboard.putNumber("Launcher Current", intakeLauncherRoller.getOutputCurrent());
        SmartDashboard.putNumber("Launcher Applied Output", intakeLauncherRoller.getAppliedOutput());
    }
}*/
////////////////////////////קוד חדש למחוק במידה ויש בעיה /////////
package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FuelConstants.*;

import org.ejml.simple.ops.WorkAroundForComplex;

public class FuelSubsystem extends SubsystemBase {

    // ================= קבועי מהירות ו-PID נעולים בקוד =================
    private final double RPM_CLOSE = 3000.0; // מהירות לירי קרוב (כפתור A)
    private final double RPM_FAR = 4000.0;   // מהירות לירי רחוק (כפתור Y)
    private final double RPM_DELIEVERY = 6000.0;

    private final double SPIN_UP_FEEDER_RPM = 2000.0;
    private final double SPIN_UP_INTAKE_RPM = -4500.0;

    // ערכי ה-PID של היורה (נעולים בקוד במקום מהדשבורד)
    private final double SHOOTER_KP = 0.14;
    private final double SHOOTER_KI = 0.0;
    private final double SHOOTER_KD = 0.0;
    private final double SHOOTER_KV = 0.122;

    private final SparkMax feederRoller;
    private final SparkMax intakeLauncherRoller;
    private final TalonFX shooterRoller;

    private final RelativeEncoder feederEncoder;
    private final RelativeEncoder intakeEncoder;

    private final SparkClosedLoopController feederPID;
    private final SparkClosedLoopController intakePID;

    private final VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0);

    private final edu.wpi.first.wpilibj.Timer pulseTimer = new edu.wpi.first.wpilibj.Timer();
    private boolean pulseCycleStarted = false;

    private double intakeSetpoint = 0;
    private double feederSetpoint = 0;
    private double shooterSetpoint = 0;

    private boolean reached_speed = false;

    public FuelSubsystem() {

        intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
        feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
        shooterRoller = new TalonFX(SHOOTER_ID);

        feederEncoder = feederRoller.getEncoder();
        intakeEncoder = intakeLauncherRoller.getEncoder();

        feederPID = feederRoller.getClosedLoopController();
        intakePID = intakeLauncherRoller.getClosedLoopController();

        configureMotors();
        
        // מציג בדשבורד את המהירויות הקבועות ליופי בלבד
        SmartDashboard.putNumber("Target Feeder RPM", 3000);
        SmartDashboard.putNumber("Target Intake RPM", 3000);
        SmartDashboard.putNumber("Spin-up Feeder RPM", 2000);

        //SmartDashboard.putData("Launch", new InstantCommand(this::launch));
    }

    private void configureMotors() {
        // ================= FEEDER =================
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
        feederConfig.closedLoop
                .p(0.0002)
                .i(0.0)
                .d(0.0)
                .velocityFF(0.00018)
                .outputRange(-1.0, 1.0);
        feederConfig.closedLoopRampRate(0.15);

        feederRoller.configure(
                feederConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // ================= INTAKE LAUNCHER =================
        SparkMaxConfig launcherConfig = new SparkMaxConfig();
        launcherConfig.inverted(true);
        launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
        launcherConfig.closedLoop
                .p(0.00022)
                .i(0.0)
                .d(0.00001)
                .velocityFF(0.00019)
                .outputRange(-1.0, 1.0);
        launcherConfig.closedLoopRampRate(0.2);

        intakeLauncherRoller.configure(
                launcherConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // ================= TALON SHOOTER PID =================
        configureShooterPID();
    }

    private void configureShooterPID() {
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        
        // טעינת ערכי ה-PID מהקבועים של המחלקה
        talonConfig.Slot0.kP = SHOOTER_KP;
        talonConfig.Slot0.kI = SHOOTER_KI;
        talonConfig.Slot0.kD = SHOOTER_KD;
        talonConfig.Slot0.kV = SHOOTER_KV;

        shooterRoller.getConfigurator().apply(talonConfig);
    }

    public void setPIDF() {
        // עידן היה פה
    }

    public void setTargetRPM(double feederRPM, double intakeRPM) {
        feederSetpoint = feederRPM;
        intakeSetpoint = intakeRPM;
        shooterSetpoint = intakeRPM;

        feederPID.setReference(
                feederRPM,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0);

        intakePID.setReference(
                intakeRPM,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0);

        shooterRoller.setControl(
                shooterVelocityRequest.withVelocity(intakeRPM / 60.0));
    }

    public void intake() {
        feederRoller.setVoltage(-INTAKING_FEEDER_VOLTAGE);
        intakeLauncherRoller.setVoltage(-INTAKING_INTAKE_VOLTAGE);
    }

    public void revIntake() {
        feederRoller.setVoltage(INTAKING_FEEDER_VOLTAGE);
        intakeLauncherRoller.setVoltage(INTAKING_INTAKE_VOLTAGE);
    }

    public void warmUpRoller(){
        shooterRoller.setControl(shooterVelocityRequest.withVelocity(RPM_CLOSE / 60.0));
        
    }


    public void feedUp(){
        
        intakePID.setReference(-RPM_CLOSE, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        feederPID.setReference(-RPM_CLOSE, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    // מנוע הפולסים החכם שמקבל את מהירות היעד ישירות מהפקודה שנבחרה
    public void eject(double ejectRPM) {
        shooterSetpoint = ejectRPM;

        // 1. הפעלת מנוע היורה (TalonFX) במהירות המבוקשת
        shooterRoller.setControl(shooterVelocityRequest.withVelocity(ejectRPM / 60.0));

        // 2. בדיקה האם היורה הגיע למהירות שלו
        if (launcherAtSpeed(ejectRPM)) {
            if (!pulseCycleStarted) {
                pulseTimer.restart();
                pulseCycleStarted = true;
            }

            // 3. מנוע הפולסים: 0.15 שניות דחיפה, 0.30 שניות עצירה
            if (pulseTimer.get() < 0.15) {
                reached_speed = true;
                intakeSetpoint = ejectRPM;
                feederSetpoint = ejectRPM;

                intakePID.setReference(-ejectRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
                feederPID.setReference(-ejectRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
            } else if (pulseTimer.get() < 0.45) {
                feederRoller.stopMotor();
                intakeLauncherRoller.stopMotor();
                feederSetpoint = 0;
                intakeSetpoint = 0;
            } else {
                pulseTimer.restart();
            }

        } else {
            pulseCycleStarted = false;
            pulseTimer.stop();

            feederSetpoint = 0;
            feederRoller.stopMotor();
            intakeLauncherRoller.stopMotor();
            intakeSetpoint = 0;
        }
    }

    public void launch() {
        setTargetRPM(-3000, 3000);
    }

    public void spinUp() {
        setTargetRPM(SPIN_UP_FEEDER_RPM, SPIN_UP_INTAKE_RPM);
    }

    public boolean launcherAtSpeed(double targetRPM) {
        double currentRPM = shooterRoller.getVelocity().getValueAsDouble() * 60.0;
        double error = Math.abs(currentRPM - targetRPM);

        if (reached_speed) {
            if (error < targetRPM * 0.05) {
                return true;
            } else {
                reached_speed = false;
                return false;
            }
        }

        if (error < targetRPM * 0.03) {
            reached_speed = true;
            return true;
        }

        return false;
    }

    public void stopRoller() {
        shooterRoller.stopMotor();

        shooterSetpoint = 0;
    }

    public void stopIntake(){
        feederRoller.stopMotor();
        intakeLauncherRoller.stopMotor();

        feederSetpoint = 0;
        intakeSetpoint = 0;

        pulseCycleStarted = false;
        pulseTimer.stop();
        pulseTimer.reset();
    }

     public void stop(){
        stopIntake();
        stopRoller();
    }


    public Command spinUpCommand() {
        return this.run(this::spinUp);
    }

    // ================= פקודות חדשות עבור השלט (נקראות מרובוט קונטיינר) =================

    /**
     * פקודה לירי קרוב (3000 סל"ד)
     */
    public Command launchCommand() {
        return this.run(() -> this.eject(RPM_CLOSE)).finallyDo(this::stop);
    }

    /**
     * פקודה לירי רחוק (4000 סל"ד בדיוק)
     */
    public Command launchFarCommand() {
        return this.run(() -> this.eject(RPM_FAR)).finallyDo(this::stop);
    }

    //פקודה לירי מהמרכז ( 6000 - Delievery)
    public Command launchDelieveryCommand() {
        return this.run( () -> this.eject(RPM_DELIEVERY)).finallyDo(this::stop);
    } 

    @Override
    public void periodic() {
        // שידור נתונים בלבד למסך של הנהגים (אין קריאות חוזרות מהדשבורד)
        SmartDashboard.putNumber("Feeder Velocity", feederEncoder.getVelocity());
        SmartDashboard.putNumber("Launcher Velocity", intakeEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter Roller RPM", shooterRoller.getVelocity().getValueAsDouble() * 60.0);
        SmartDashboard.putNumber("Launcher Setpoint", intakeSetpoint);
        SmartDashboard.putNumber("Shooter Setpoint", shooterSetpoint);
        SmartDashboard.putNumber("Launcher Error", intakeSetpoint - intakeEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter Error", shooterSetpoint - shooterRoller.getVelocity().getValueAsDouble() * 60.0);
        SmartDashboard.putBoolean("Launcher Ready", launcherAtSpeed(shooterSetpoint));
    }
}