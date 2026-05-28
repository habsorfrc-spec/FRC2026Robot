package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FuelConstants.*;

public class BackIntakeSubsystem extends SubsystemBase {

    private final SparkMax backIntakeLimb;
    private final WPI_VictorSPX backIntakeRoller;

    private final RelativeEncoder limbEncoder;
    private final SparkClosedLoopController limbPID;
    private final DigitalInput limitSwitch;

    private static final double LIMB_UP_POSITION = 0.0;
    private static final double LIMB_DOWN_POSITION = 0.635;

    // Strong PID gains for fast movement
    private static final double LIMB_kP = 500.0;
    private static final double LIMB_kD = 5;

    private static final double POSITION_TOLERANCE = 0.01;

    private double limbTarget = LIMB_UP_POSITION;
    private boolean motorActive = false;

    public BackIntakeSubsystem() {
        backIntakeLimb = new SparkMax(BACK_INTAKE_LIMB_MOTOR_ID, MotorType.kBrushed);
        backIntakeRoller = new WPI_VictorSPX(BACK_INTAKE_MOTOR_ID);
        limitSwitch = new DigitalInput(0);

        limbEncoder = backIntakeLimb.getEncoder();
        limbPID = backIntakeLimb.getClosedLoopController();

        SmartDashboard.putNumber("Limb Position", 0);

        SmartDashboard.putNumber("Get Voltage", backIntakeLimb.getBusVoltage());
        SmartDashboard.putNumber("Get Output Current", backIntakeLimb.getOutputCurrent());

        SparkMaxConfig limbConfig = new SparkMaxConfig();
        limbConfig.idleMode(IdleMode.kBrake);
        limbConfig.smartCurrentLimit(80);

        limbConfig.closedLoop
                .p(LIMB_kP)
                .d(LIMB_kD)
                .outputRange(-1.0, 1.0);

        backIntakeLimb.configure(limbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Move limb to a target position */
    public void moveLimbToPosition(double position) {
        limbTarget = Math.max(LIMB_UP_POSITION, Math.min(LIMB_DOWN_POSITION, position));
        motorActive = true; // activate PID until target reached
    }

   /** Call periodically to update PID and stop when target is reached */
private void updateLimb() {
    if (!motorActive) {
        return;
    }

    double current = limbEncoder.getPosition();
    double error = limbTarget - current;

    boolean atTop = !limitSwitch.get();

    // Zero encoder when top limit switch is pressed
    if (atTop) {
        limbEncoder.setPosition(0);
    }

    // Stop when target reached
    if (Math.abs(error) < POSITION_TOLERANCE) {
        backIntakeLimb.stopMotor();
        motorActive = false;
        return;
    }

    // Prevent driving upward into the hard stop
    if (limbTarget == LIMB_UP_POSITION && atTop) {
        backIntakeLimb.stopMotor();
        motorActive = false;
        return;
    }

    // Continue PID movement
    limbPID.setReference(
        limbTarget,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0
    );
}
    public void limbUp() {
        moveLimbToPosition(LIMB_UP_POSITION);
    }

    public void limbDown() {
        moveLimbToPosition(LIMB_DOWN_POSITION);
    }

    /** Run back intake rollers forward */
    public void BackIntake() {
        double percent = SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE) / 12.0;
        backIntakeRoller.set(ControlMode.PercentOutput, percent);
    }

    /** Run back intake rollers reversed */
    public void revBackIntake() {
        double percent = -SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE) / 12.0;
        backIntakeRoller.set(ControlMode.PercentOutput, percent);
    }

    /** Stop all motors */
    public void stop() {
        backIntakeRoller.stopMotor();
        backIntakeLimb.stopMotor();
        motorActive = false;
    }

    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("Limb Position", limbEncoder.getPosition());
        SmartDashboard.putBoolean("Limit Switch Pressed", !limitSwitch.get());
        
        SmartDashboard.putNumber("Get Voltage", backIntakeLimb.getBusVoltage());
        SmartDashboard.putNumber("Get Output Current", backIntakeLimb.getOutputCurrent());
        
        // Update limb PID only while motor is active
        updateLimb();
    }
}