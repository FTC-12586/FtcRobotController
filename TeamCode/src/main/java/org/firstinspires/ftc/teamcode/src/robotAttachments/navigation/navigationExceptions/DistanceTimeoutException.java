package org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.LocalizationAlgorithm;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtils;

/**
 * Tests to see if the robot has remained stationary for too long
 */
public class DistanceTimeoutException extends MovementException {

    /**
     * Timer to keep track of idle time
     */
    private final ElapsedTime timer;
    /**
     * The amount of acceptable distance to travel within the loop time frame
     */
    private final double tooSmallOfDistance; // this travels ~2 inches for every 1000 millis
    /**
     * The length of each time frame
     */
    private final double millis;
    /**
     * The position the robot is in after the loop
     */
    private double positionAfterTimeLoop = Double.MAX_VALUE; //These are arrays to make the compiler happy. Treat them as a normal double

    /**
     * A constructor
     *
     * @param millis The length of each time frame that this is run in milliseconds
     */
    public DistanceTimeoutException(double millis) {
        super();
        timer = new ElapsedTime();
        tooSmallOfDistance = millis / 500.0; // this travels ~2 inches for every 1000 millis
        this.millis = millis;

    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void call(double x, double y, double theta, double tolerance, Telemetry telemetry, LocalizationAlgorithm gps, Executable<Boolean> _isStopRequested, Executable<Boolean> _opModeIsActive, VoltageSensor voltageSensor) throws MovementException {
        if (timer.milliseconds() >= millis) {
            //These are arrays to make the compiler happy. Treat them as a normal double
            double positionBeforeTimeLoop = positionAfterTimeLoop;
            positionAfterTimeLoop = MiscUtils.distance(gps.getX(), gps.getY(), x, y);
            double traveledDistance = Math.abs(positionBeforeTimeLoop - positionAfterTimeLoop);
            if (traveledDistance < tooSmallOfDistance) {
                final String args = "moveToPosition(" + x + ", " + y + ", " + theta + ", " + tolerance + ")\n";
                final String errorMsg = "In function call " + args + MiscUtils.getRelativeClassName(this) + " Exception.\n";
                RobotLog.addGlobalWarningMessage(errorMsg);
                throw this;
            }
            timer.reset();
        }
    }
}
