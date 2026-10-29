package frc.robot.subsystems;
import edu.wpi.first.math.controller.*;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand; // for the commented-out "Launch" line in the constructor
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FuelConstants.*;

public class FuelSubsystem extends SubsystemBase {
    private boolean isStoped = true;

    // ================= קבועי מהירות ו-PID נעולים בקוד =================
    private final double RPM_CLOSE = 3500; // מהירות לירי קרוב (כפתור A)
    private final double RPM_FAR = 4000.0; // מהירות לירי רחוק (כפתור Y)
    private final double RPM_DELIEVERY = 6000.0;

    private final double SPIN_UP_FEEDER_RPM = 4000.0;
    private final double SPIN_UP_INTAKE_RPM = -4500.0;

     private final double KP = 0.1;
      private final double KI = 0.0;
       private final double KD = 0.001;
        private final double KV = 0.113;

    private double volts = 0.0;

    // ערכי ה-PID של היורה (נעולים בקוד במקום מהדשבורד)
    private double SHOOTER_KP = SmartDashboard.getNumber("Kp", KP);
    private  double SHOOTER_KI = SmartDashboard.getNumber("Ki", KI);
    private  double SHOOTER_KD = SmartDashboard.getNumber("Kd", KD);
    private  double SHOOTER_KV = SmartDashboard.getNumber("Kv", KV);

    private final SparkMax feederRoller;
    private final SparkMax intakeLauncherRoller;
    private final TalonFX shooterRoller;
    private final TalonFX shooterFollower;

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
        shooterFollower = new TalonFX(SHOOTER_FOLLOWER_ID);

        feederEncoder = feederRoller.getEncoder();
        intakeEncoder = intakeLauncherRoller.getEncoder();

        feederPID = feederRoller.getClosedLoopController();
        intakePID = intakeLauncherRoller.getClosedLoopController();

        configureMotors();

        // מציג בדשבורד את המהירויות הקבועות ליופי בלבד
        SmartDashboard.putNumber("Target Feeder RPM", 3000);
        SmartDashboard.putNumber("Target Intake RPM", 3000);
        SmartDashboard.putNumber("Spin-up Feeder RPM", 2000);
        

        SmartDashboard.putNumber("ROLLER VOLTS", 6.0);

        SmartDashboard.putNumber("Kp", KP);
        SmartDashboard.putNumber("Ki", KI);
        SmartDashboard.putNumber("Kd", KD);
        SmartDashboard.putNumber("Kv", KV);

        // SmartDashboard.putData("Launch", new InstantCommand(this::launch));
    }

    // ================= SETUP =================

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

        // NEW: follower setup
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        shooterFollower.getConfigurator().apply(followerConfig);

        // If the follower spins the SAME physical direction as the master:
        shooterFollower.setControl(new Follower(SHOOTER_ID, MotorAlignmentValue.Aligned));

        // If it's mounted to spin the OPPOSITE direction (common on dual-roller shooters
        // where one wheel needs to spin backward relative to the other to work together):
        shooterFollower.setControl(new Follower(SHOOTER_ID, MotorAlignmentValue.Opposed));
    }

   private void configureShooterPID() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = SHOOTER_KP;
    config.Slot0.kI = SHOOTER_KI;
    config.Slot0.kD = SHOOTER_KD;
    config.Slot0.kV = SHOOTER_KV;

    config.Voltage.PeakForwardVoltage = 10.5; // Scale math to 10.5V max
    config.Voltage.PeakReverseVoltage = -10.5; 

    shooterRoller.getConfigurator().apply(config);
}

    public void setPIDF() {
        // עידן היה פה
    }

    // ================= MOTOR ACTIONS =================

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

    
    public void warmUpRoller(double rpm) {
        shooterSetpoint = rpm;
        
        //volts = SmartDashboard.getNumber("ROLLER VOLTS", 6.0);

        shooterRoller.setControl(
             shooterVelocityRequest.withVelocity(rpm / 60.0)
         );

//    shooterRoller.setControl(
//         new VoltageOut(volts)
//     );
    }


    public void feedUp() {

        intakePID.setReference(-5500, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        feederPID.setReference(-5500, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
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

    // ================= SPEED CHECK =================

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

    // ================= STOP =================

    public void stopRoller() {
        shooterRoller.stopMotor();

        shooterSetpoint = 0;
    }

    public void stopIntake() {
        feederRoller.stopMotor();
        intakeLauncherRoller.stopMotor();

        feederSetpoint = 0;
        intakeSetpoint = 0;

        pulseCycleStarted = false;
        pulseTimer.stop();
        pulseTimer.reset();
    }

    public void stop() {
        stopIntake();
        stopRoller();
    }

    // ================= COMMANDS =================

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

    // פקודה לירי מהמרכז ( 6000 - Delievery)
    public Command launchDelieveryCommand() {
        return this.run(() -> this.eject(RPM_DELIEVERY)).finallyDo(this::stop);
    }

    // POV Right: shooter warm-up on/off
    public Command warmUpStopToggleCommand(double rpm) {
        return new FunctionalCommand(
                () -> {
                    if (isStoped)
                        warmUpRoller(rpm);
                    else
                        stopRoller();
                },
                () -> {},
                interupted -> {},
                () -> launcherAtSpeed(RPM_CLOSE), this)
                .finallyDo(() -> {
                    isStoped = !isStoped;
                });
    }

    // ================= DASHBOARD =================

    @Override
    public void periodic() {
        // שידור נתונים בלבד למסך של 5הנהגים (אין קריאות חוזרות מהדשבורד)
        SmartDashboard.putNumber("Feeder Velocity", feederEncoder.getVelocity());
        SmartDashboard.putNumber("Launcher Velocity", intakeEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter Roller RPM", shooterRoller.getVelocity().getValueAsDouble() * 60.0);
        SmartDashboard.putNumber("Launcher Setpoint", intakeSetpoint);
        SmartDashboard.putNumber("Shooter Setpoint", shooterSetpoint);
        SmartDashboard.putNumber("Launcher Error", intakeSetpoint - intakeEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter Error",
                shooterSetpoint - shooterRoller.getVelocity().getValueAsDouble() * 60.0);
        SmartDashboard.putBoolean("Launcher Ready", launcherAtSpeed(shooterSetpoint));
        SmartDashboard.putNumber("Roller Voltage", shooterRoller.getMotorVoltage().getValueAsDouble());

          // === DIAGNOSTIC CHECK FOR LIMITS ===
    // Check if the Stator (Current) Limit is actively blocking voltage
    boolean isCurrentLimiting = shooterRoller.getFault_StatorCurrLimit().getValue();
    SmartDashboard.putBoolean("Shooter Limit Active/Fault", isCurrentLimiting);

    // Check the raw Duty Cycle (How close to 100% max hardware capacity the motor is)
    double dutyCycle = shooterRoller.getDutyCycle().getValueAsDouble();
    SmartDashboard.putNumber("Shooter Duty Cycle", dutyCycle);

    // Check the actual supply voltage reaching the controller right now
    double supplyVolts = shooterRoller.getSupplyVoltage().getValueAsDouble();
    SmartDashboard.putNumber("Shooter Input Volts", supplyVolts);

        

           // ערכי ה-PID של היורה (נעולים בקוד במקום מהדשבורד)
        // Update shooter PID values from SmartDashboard
double kP = SmartDashboard.getNumber("Kp", KP);
double kI = SmartDashboard.getNumber("Ki", KI);
double kD = SmartDashboard.getNumber("Kd", KD);
double kV = SmartDashboard.getNumber("Kv", KV);

        if (kP != SHOOTER_KP || kI != SHOOTER_KI ||
    kD != SHOOTER_KD || kV != SHOOTER_KV) {

    SHOOTER_KP = kP;
    SHOOTER_KI = kI;
    SHOOTER_KD = kD;
    SHOOTER_KV = kV;

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = SHOOTER_KP;
    config.Slot0.kI = SHOOTER_KI;
    config.Slot0.kD = SHOOTER_KD;
    config.Slot0.kV = SHOOTER_KV;



    shooterRoller.getConfigurator().apply(config);
}
    }
}