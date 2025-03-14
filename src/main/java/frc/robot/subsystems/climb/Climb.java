package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Climb extends SubsystemBase {
    private final ClimbIO io;

    public Climb(ClimbIO io) {
        this.io = io;
        io.disableFlipServo();
        io.setClimbMotorOutput(0.0);
        io.setClimbRotationMotorOutput(0.0);
    }

    public Command climbCommand(DoubleSupplier climbPowerSupplier, DoubleSupplier climbRotationPowerSupplier) {
        return run(() -> {
                    io.setClimbMotorOutput(climbPowerSupplier.getAsDouble() * 12.0);
                    io.setClimbRotationMotorOutput(climbRotationPowerSupplier.getAsDouble() * 12.0);
                })
                .beforeStarting(() -> io.setFlipServo(true))
                .finallyDo(() -> {
                    io.setClimbMotorOutput(0.0);
                    io.setClimbRotationMotorOutput(0.0);
                });
    }

    public Command cancelClimb() {
        return runOnce(() -> io.setFlipServo(false))
                .andThen(Commands.waitSeconds(0.5))
                .finallyDo(io::disableFlipServo);
    }

    public interface ClimbIO {
        default void setClimbMotorOutput(double volts) {}

        default void setClimbRotationMotorOutput(double volts) {}

        default void setFlipServo(boolean activated) {}

        default void disableFlipServo() {}
    }

    public static final class ClimbIOReal implements ClimbIO {
        private final Servo servo;
        private final TalonFX climbMotor;
        private final TalonFX climbRotationMotor;

        public ClimbIOReal() {
            this.servo = new Servo(1);
            servo.setDisabled();
            this.climbMotor = new TalonFX(17);
            this.climbRotationMotor = new TalonFX(16);
            climbMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
            climbMotor
                    .getConfigurator()
                    .apply(new CurrentLimitsConfigs()
                            .withSupplyCurrentLimitEnable(true)
                            .withSupplyCurrentLimit(40)
                            .withStatorCurrentLimitEnable(true)
                            .withStatorCurrentLimit(60));

            climbRotationMotor
                    .getConfigurator()
                    .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
            climbRotationMotor
                    .getConfigurator()
                    .apply(new CurrentLimitsConfigs()
                            .withSupplyCurrentLimitEnable(true)
                            .withSupplyCurrentLimit(40)
                            .withStatorCurrentLimitEnable(true)
                            .withStatorCurrentLimit(60));
        }

        private final VoltageOut voltageOut = new VoltageOut(0);

        @Override
        public void setClimbMotorOutput(double volts) {
            climbMotor.setControl(voltageOut.withOutput(volts));
        }

        @Override
        public void setClimbRotationMotorOutput(double volts) {
            climbMotor.setControl(voltageOut.withOutput(volts));
        }

        @Override
        public void setFlipServo(boolean activated) {
            servo.set(activated ? 0.2 : 0.7);
        }

        @Override
        public void disableFlipServo() {
            servo.setDisabled();
        }
    }
}
