package frc.lib;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;

public class CTREUtil {
    public static StatusCode tryUntilOK(Supplier<StatusCode> function, int deviceId) {
        final int max_num_retries = 10;
        StatusCode statusCode = StatusCode.OK;
        for (int i = 0; i < max_num_retries; ++i) {
            statusCode = function.get();
            if (statusCode == StatusCode.OK) break;
        }
        if (statusCode != StatusCode.OK) {
            DriverStation.reportError(
                    "Error calling " + function + " on ctre device id " + deviceId + ": " + statusCode, true);
        }
        return statusCode;
    }

    public static StatusCode applyConfiguration(TalonFX motor, TalonFXConfiguration config) {
        return tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static StatusCode applyConfiguration(CANrange sensor, CANrangeConfiguration config) {
        return tryUntilOK(() -> sensor.getConfigurator().apply(config), sensor.getDeviceID());
    }

    public static StatusCode applyConfiguration(CANcoder cancoder, CANcoderConfiguration config) {
        return tryUntilOK(() -> cancoder.getConfigurator().apply(config), cancoder.getDeviceID());
    }

    public static StatusCode applyConfiguration(ParentDevice device, ParentConfiguration config) {
        if (device instanceof TalonFX) return applyConfiguration((TalonFX) device, (TalonFXConfiguration) config);
        else if (device instanceof CANcoder) return applyConfiguration((CANcoder) device, (CANcoderConfiguration) config);
        else if (device instanceof CANrange) return applyConfiguration((CANrange) device, (CANrangeConfiguration) config);
        throw new IllegalArgumentException("Device type not supported");
    }

    public static StatusCode refreshConfiguration(TalonFX motor, TalonFXConfiguration config) {
        return tryUntilOK(() -> motor.getConfigurator().refresh(config), motor.getDeviceID());
    }
}
