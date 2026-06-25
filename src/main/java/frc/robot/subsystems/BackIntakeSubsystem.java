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
    private final DigitalInput limitSwitch;
    private final SparkClosedLoopController pid;

    private static final double TOP_POSITION = 0;
    private static final double BOTTOM_POSITION = 0.6;



    public BackIntakeSubsystem() {
        backIntakeLimb = new SparkMax(BACK_INTAKE_LIMB_MOTOR_ID, MotorType.kBrushed);
        backIntakeRoller = new WPI_VictorSPX(BACK_INTAKE_MOTOR_ID);
        limitSwitch = new DigitalInput(0);

        pid = backIntakeLimb.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();
        
        config.closedLoop
            .p(8.0)
            .d(0.5)
            .outputRange(-0.9, 0.9);

        backIntakeLimb.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        limbEncoder = backIntakeLimb.getEncoder();

        SmartDashboard.putNumber("Limb Position", 0);
        SmartDashboard.putNumber("Get Voltage", backIntakeLimb.getBusVoltage());
        SmartDashboard.putNumber("Get Output Current", backIntakeLimb.getOutputCurrent());

    }

    public void limbUp() {
        pid.setReference(
            TOP_POSITION,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0
    );

}

    public void limbDown() {
        pid.setReference(
            BOTTOM_POSITION,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0
    );
}

    /**
     * Run back intake rollers forward at configured voltage
     */
    public void BackIntake() {
        double percent = SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE) / 12.0;
        backIntakeRoller.set(ControlMode.PercentOutput, percent);
    }

    /**
     * Run back intake rollers in reverse at configured voltage
     */
    public void revBackIntake() {
        double percent = -SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE) / 12.0;
        backIntakeRoller.set(ControlMode.PercentOutput, percent);
    }

    /**
     * Stop all motors immediately and cut power
     */
    public void stop() { 
        backIntakeLimb.set(0);
        backIntakeRoller.stopMotor();
        backIntakeLimb.stopMotor();
    }

    @Override
    public void periodic() {
        // SmartDashboard telemetry for debugging and tuning
        SmartDashboard.putNumber("Limb Position", limbEncoder.getPosition());
        SmartDashboard.putBoolean("Limit Switch Pressed", !limitSwitch.get());

        SmartDashboard.putNumber("Get Voltage", backIntakeLimb.getBusVoltage());
        SmartDashboard.putNumber("Get Output Current", backIntakeLimb.getOutputCurrent());
    }
}