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
        SmartDashboard.putNumber("Shooter kP", 0.12);
        SmartDashboard.putNumber("Shooter kI", 0.0);
        SmartDashboard.putNumber("Shooter kD", 0.0);
        SmartDashboard.putNumber("Shooter kV", 0.12);

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

        talonConfig.Slot0.kP = SmartDashboard.getNumber("Shooter kP", 0.12);
        talonConfig.Slot0.kI = SmartDashboard.getNumber("Shooter kI", 0.0);
        talonConfig.Slot0.kD = SmartDashboard.getNumber("Shooter kD", 0.0);
        talonConfig.Slot0.kV = SmartDashboard.getNumber("Shooter kV", 0.12);

        shooterRoller.getConfigurator().apply(talonConfig);
    }

    public void setPIDF() {
        // optional live tuning later
    }

    /**
     * Sets velocity PID targets for the feeder, intake launcher, and TalonFX
     * shooter.
     * shooterRPM mirrors intakeRPM — adjust if they differ.
     */
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

        // TalonFX PID via VelocityVoltage (rotations/sec)
        shooterRoller.setControl(
                shooterVelocityRequest.withVelocity(ejectRPM / 60.0));

        // Only feed once shooter is up to speed
        if (launcherAtSpeed(ejectRPM)) {
            reached_speed = true;
            intakeSetpoint = ejectRPM;
            feederSetpoint = ejectRPM;

            intakePID.setReference(
                    -ejectRPM,
                    ControlType.kVelocity,
                    ClosedLoopSlot.kSlot0);

            feederPID.setReference(
                    -ejectRPM,
                    ControlType.kVelocity,
                    ClosedLoopSlot.kSlot0);

        } else {

            feederSetpoint = 0;
            feederRoller.stopMotor();
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

        // TalonFX velocity is rotations/sec → convert to RPM for comparison
        double currentRPM = shooterRoller.getVelocity().getValueAsDouble() * 60.0;
        double d = Math.abs(currentRPM - targetRPM);

        if (reached_speed) {
            if (d < targetRPM * 0.3) {
                return true;
            } else {
                reached_speed = false;
                return false;
            }
        }

        return d < targetRPM * 0.05;
    }

    public void stop() {

        feederRoller.stopMotor();
        intakeLauncherRoller.stopMotor();
        shooterRoller.stopMotor();

        feederSetpoint = 0;
        intakeSetpoint = 0;
        shooterSetpoint = 0;
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

        double kP = SmartDashboard.getNumber("Shooter kP", 0.12);
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
}