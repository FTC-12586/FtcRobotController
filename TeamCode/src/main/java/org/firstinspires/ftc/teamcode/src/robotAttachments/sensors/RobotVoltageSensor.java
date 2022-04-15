package org.firstinspires.ftc.teamcode.src.robotAttachments.sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * A wrapper class for the Robot Voltage sensor
 */
public class RobotVoltageSensor implements VoltageSensor {
    /**
     * Internal voltage sensor object
     */
    private final VoltageSensor sensor;

    /**
     * Gets the voltage sensor from the hardware map, stores it internally
     *
     * @param hardwareMap The OpMode hardware map
     */
    public RobotVoltageSensor(HardwareMap hardwareMap) {
        this.sensor = RobotVoltageSensor.extractVoltageSensor(hardwareMap);
        assert sensor != null; //Sanity Check
    }

    /**
     * Extracts a voltage sensor from the hardware map
     *
     * @param hardwareMap The hardware map to get a voltage sensor from
     * @return A voltage sensor
     */
    public static VoltageSensor extractVoltageSensor(HardwareMap hardwareMap) {
        for (com.qualcomm.robotcore.hardware.VoltageSensor _sensor : hardwareMap.voltageSensor) {
            return _sensor;
        }
        return null;
    }

    /**
     * Returns the voltage
     *
     * @return Returns the voltage in Volts
     */
    public double getVoltage() {
        return sensor.getVoltage();
    }

    @Override
    public Manufacturer getManufacturer() {
        return sensor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return sensor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return sensor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return sensor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        sensor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        sensor.close();
    }
}
