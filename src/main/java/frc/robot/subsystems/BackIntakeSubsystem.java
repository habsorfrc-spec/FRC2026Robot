package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FuelConstants.*;

public class BackIntakeSubsystem extends SubsystemBase {

    private final TalonFX backIntakeLimb;
    private final WPI_VictorSPX backIntakeRoller;

    private final DigitalInput limitSwitch;

    private static final double TOP_POSITION = 0;
    private static final double BOTTOM_POSITION = 0.6;

    public BackIntakeSubsystem() {
        backIntakeLimb = new TalonFX(BACK_INTAKE_LIMB_MOTOR_ID);
        backIntakeRoller = new WPI_VictorSPX(BACK_INTAKE_MOTOR_ID);
        
        limitSwitch = new DigitalInput(0);

        SmartDashboard.putNumber("Limb Position", 0);

    }

    public void limbUp() {
    
        backIntakeLimb.set(-1);
        
    }

    public void limbDown() {
        
        backIntakeLimb.set(0.3);
        
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
        backIntakeRoller.stopMotor();
        backIntakeLimb.stopMotor();
    }

    @Override
    public void periodic() {

        // SmartDashboard telemetry for debugging and tuning
         SmartDashboard.putNumber("Limb Position", backIntakeLimb.getPosition().getValueAsDouble());
        // SmartDashboard.putBoolean("Limit Switch Pressed", !limitSwitch.get());
    }
}