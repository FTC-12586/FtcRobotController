package org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.LocalizationAlgorithm;
import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtils;

/**
 * A tests to see if the Distance Sensor reading falls below a certain threshold
 */
public class DistanceSensorException extends MovementException {

    /**
     * Internal DistanceSensor Object
     */
    private final DistanceSensor distanceSensor;

    /**
     * The minimum value (in centimeters) that the distance sensor reading must stay above
     */
    private final double threshold;


    /**
     * A constructor
     *
     * @param distanceSensor A distance sensor object
     * @param threshold      The minimum value (in centimeters) that the distance sensor reading must stay above
     */
    public DistanceSensorException(DistanceSensor distanceSensor, double threshold) {
        super();
        this.distanceSensor = distanceSensor;
        this.threshold = threshold;
    }

    /**
     * {@inheritDoc}
     */
    public void call(double x, double y, double theta, double tolerance, Telemetry telemetry, LocalizationAlgorithm gps, Executable<Boolean> _isStopRequested, Executable<Boolean> _opModeIsActive, RobotVoltageSensor voltageSensor) throws MovementException {
        if (this.distanceSensor.getDistance(DistanceUnit.CM) < threshold) {
            final String args = "moveToPosition(" + x + ", " + y + ", " + theta + ", " + tolerance + ")\n";
            final String errorMsg = "In function call " + args + MiscUtils.getRelativeClassName(this) + " Exception.\n";
            RobotLog.addGlobalWarningMessage(errorMsg);
            throw this;
        }
    }
}
