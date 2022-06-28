package org.firstinspires.ftc.teamcode.src.robotAttachments.sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.jetbrains.annotations.NotNull;

/**
 * Implements the Space Bar as a standard Touch Sensor
 */
public class SpaceBar implements TouchSensor {
    private final TouchSensor sensor1;

    private final TouchSensor sensor2;

    /**
     * Constructs and initializes underlying members
     *
     * @param hardwareMap An Opmode hardware map
     * @param sensor1Name The name of the first touch sensor
     * @param sensor2Name The name of the second touch sensor
     */
    public SpaceBar(final @NotNull HardwareMap hardwareMap, final @NotNull String sensor1Name, final @NotNull String sensor2Name) {
        sensor1 = hardwareMap.touchSensor.get(sensor1Name);
        sensor2 = hardwareMap.touchSensor.get(sensor2Name);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double getValue() {
        return (sensor1.getValue() + sensor2.getValue()) / 2;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean isPressed() {
        return sensor1.isPressed() || sensor2.isPressed();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Unknown;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String getDeviceName() {
        return sensor1.getDeviceName() + " and " + sensor2.getDeviceName();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String getConnectionInfo() {
        return sensor1.getConnectionInfo() + " and " + sensor2.getConnectionInfo();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public int getVersion() {
        return (sensor1.getVersion() + sensor2.getVersion()) / 2;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void resetDeviceConfigurationForOpMode() {
        sensor2.resetDeviceConfigurationForOpMode();
        sensor1.resetDeviceConfigurationForOpMode();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void close() {
        sensor2.close();
        sensor1.close();
    }

}
