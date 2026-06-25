package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElevatorConstants.*;

public class ElevatorSubsystem extends SubsystemBase {

    private final TalonFX leader = new TalonFX(ELEVATOR_MOTOR_ID);
    private final TalonFX follower = new TalonFX(ELEVATOR_MOTOR_FOLLOWER_ID);

    private final DutyCycleOut duty = new DutyCycleOut(0);

    public ElevatorSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        leader.getConfigurator().apply(config);
        follower.getConfigurator().apply(config);

        // Reset encoder
        leader.setPosition(0);

        // Brake mode (important for elevators)
        leader.setNeutralMode(NeutralModeValue.Brake);
        follower.setNeutralMode(NeutralModeValue.Brake);
    }

    public void elevateUp() {
        double pos = leader.getPosition().getValueAsDouble();

        if (pos < 30) { // upper limit
            leader.setControl(duty.withOutput(0.6));
            follower.setControl(duty.withOutput(0.6));
        } else {
            stop(); // IMPORTANT
        }
    }

    public void elevateDown() {
        double pos = leader.getPosition().getValueAsDouble();

        if (pos > -1) { // lower limit
            leader.setControl(duty.withOutput(-0.6));
            follower.setControl(duty.withOutput(-0.6));
        } else {
            stop(); // IMPORTANT
        }
    }

    @Override
    public void periodic() {
        double pos = leader.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Elev pos", pos);
    }

    public void stop() {
        leader.setControl(duty.withOutput(0));
        follower.setControl(duty.withOutput(0));
    }
}