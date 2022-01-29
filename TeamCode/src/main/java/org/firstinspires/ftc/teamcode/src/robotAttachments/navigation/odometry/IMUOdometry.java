package org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.LocalizationAlgorithm;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.odometry.enums.FieldPoints;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.ThreadedSubsystemTemplate;

import java.io.File;


/**
 * A class to do position triangulation based on dead wheel odometry and the BNO055IMU
 *
 * @author Sarthak
 * @since 6/1/2019
 */
public class IMUOdometry extends ThreadedSubsystemTemplate implements LocalizationAlgorithm {
    /**
     * The Vertical Left Odometry Encoder
     */
    private final DcMotor verticalEncoderLeft;
    /**
     * The Vertical Right Odometry Encoder
     */
    private final DcMotor verticalEncoderRight;
    /**
     * The Horizontal Odometry Encoder
     */
    private final DcMotor horizontalEncoder;

    /**
     * The number of encoder ticks per linear inch
     */
    private final double COUNTS_PER_INCH = 1892.3724283364;
    /**
     * An Algorithm constant
     */
    private final double robotEncoderWheelDistance;

    //Algorithm constants
    /**
     * An Algorithm constant
     */
    private final double horizontalEncoderTickPerDegreeOffset;
    /**
     * The IMU for greater angle precision
     */
    private final BNO055IMU imu;
    /**
     * An Algorithm constant
     */
    private volatile double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;
    /**
     * An Algorithm constant
     */
    private double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;
    /**
     * An Algorithm constant
     */
    private int verticalLeftEncoderPositionMultiplier = 1;
    /**
     * An Algorithm constant
     */
    private int verticalRightEncoderPositionMultiplier = 1;
    /**
     * An Algorithm constant
     */
    private int normalEncoderPositionMultiplier = 1;
    /**
     * A internal variable used to set orientation of the IMU. Rather than reset the IMU, we record the offset between the given angle and the desired angle and apply that when we get the IMU angle
     */
    private double angleOffset = 0;


    /**
     * Constructor for GlobalCoordinatePosition Thread
     *
     * @param verticalEncoderLeft  left odometry encoder, facing the vertical direction
     * @param verticalEncoderRight right odometry encoder, facing the vertical direction
     * @param horizontalEncoder    horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     * @param threadSleepDelay     delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
     * @param _isStopRequested     A Executable object wrapped around {@link LinearOpMode#isStopRequested()}
     * @param _opModeIsActive      A Executable object wrapped around {@link LinearOpMode#opModeIsActive()}
     */
    public IMUOdometry(DcMotor verticalEncoderLeft, DcMotor verticalEncoderRight, DcMotor horizontalEncoder, BNO055IMU imu, int threadSleepDelay, Executable<Boolean> _opModeIsActive, Executable<Boolean> _isStopRequested) {
        super(_opModeIsActive, _isStopRequested);
        this.verticalEncoderLeft = verticalEncoderLeft;
        this.verticalEncoderRight = verticalEncoderRight;
        this.horizontalEncoder = horizontalEncoder;
        sleepTime = threadSleepDelay;

        this.imu = imu;

        //Files to access the algorithm constants
        File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");
        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());

    }


    /**
     * Returns the ports the encoders are plugged into for debug purposes
     *
     * @return the ports in the form Vertical Right, Vertical Left, Horizontal
     */
    public int[] getPorts() {
        int[] ports = new int[3];
        ports[0] = verticalEncoderRight.getPortNumber();
        ports[1] = verticalEncoderLeft.getPortNumber();
        ports[2] = horizontalEncoder.getPortNumber();
        return ports;
    }

    /**
     * Outputs the location to the telemetry
     *
     * @param telemetry The OpMode telemetry
     */
    public void showPosition(Telemetry telemetry) {
        telemetry.addData("X: ", this.getX());
        telemetry.addData("Y: ", this.getY());
        telemetry.addData("Rotation: ", this.getRot());
    }

    /**
     * Getter for the COUNTS_PER_INCH variable
     *
     * @return COUNTS_PER_INCH in ticks per inch
     */
    public double getCOUNTS_PER_INCH() {
        return COUNTS_PER_INCH;
    }

    /**
     * Getter for the right encoder position
     *
     * @return The right encoder position in ticks
     */
    public int returnRightEncoderPosition() {
        return verticalRightEncoderPositionMultiplier * verticalEncoderRight.getCurrentPosition();
    }

    /**
     * Getter for the left encoder position
     *
     * @return The left encoder position in ticks
     */
    public int returnLeftEncoderPosition() {
        return verticalLeftEncoderPositionMultiplier * verticalEncoderLeft.getCurrentPosition();
    }

    /**
     * Getter for the horizontal encoder position
     *
     * @return The horizontal encoder position in ticks
     */
    public int returnHorizontalEncoderPosition() {
        return normalEncoderPositionMultiplier * horizontalEncoder.getCurrentPosition();
    }

    /**
     * Returns the tick values of the encoders
     *
     * @return Returns in this order: Vertical Right, Vertical Left, Horizontal
     */
    public int[] returnRaw() {
        int[] positions = new int[3];
        positions[0] = verticalEncoderRight.getCurrentPosition();
        positions[1] = verticalEncoderLeft.getCurrentPosition();
        positions[2] = horizontalEncoder.getCurrentPosition();
        return positions;
    }

    /**
     * Sets the orientation of the odometry calibration system
     *
     * @param angle the angle to be at
     */
    public void setOrientation(double angle) {
        synchronized (this) {
            angleOffset = Math.toRadians(angle);
        }
    }

    /**
     * Sets the position of the robot
     *
     * @param X   The X-position in inches
     * @param Y   The Y-position in inches
     * @param rot The rot in degrees
     */
    public void setPos(double X, double Y, double rot) {
        synchronized (this) {
            robotGlobalXCoordinatePosition = X * COUNTS_PER_INCH;
            robotGlobalYCoordinatePosition = Y * COUNTS_PER_INCH;
            setOrientation(rot);
        }

    }

    /**
     * Sets the position of the robot using an Enum key from FieldPoints
     *
     * @param initPos the enum key of a three value array of an init position
     */
    public void setPos(FieldPoints initPos) {
        double[] tmp = FieldPoints.positionsAndPoints.get(initPos);
        assert (tmp != null);
        synchronized (this) {
            robotGlobalXCoordinatePosition = tmp[0] * COUNTS_PER_INCH;
            robotGlobalYCoordinatePosition = tmp[1] * COUNTS_PER_INCH;
            setOrientation(tmp[2]);
        }

    }


    /**
     * Gets the current angle of the IMU
     *
     * @return The angle parallel to the floor in degrees
     */
    protected double getImuAngle() {
        double returnVal;
        if (imu.getAngularOrientation().firstAngle < 0) {
            returnVal = Math.abs(imu.getAngularOrientation().firstAngle);
        } else {
            returnVal = Math.abs(imu.getAngularOrientation().firstAngle - 360);
        }
        return returnVal % 360;

    }


    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    public void threadMain() {
        synchronized (this) {

            robotOrientationRadians = Math.toRadians(getImuAngle()) + angleOffset;
            robotOrientationRadians = robotOrientationRadians % 360;

            //Get Current Positions
            double verticalLeftEncoderWheelPosition = (verticalEncoderLeft.getCurrentPosition() * verticalLeftEncoderPositionMultiplier);
            //Position variables used for storage and calculations
            double verticalRightEncoderWheelPosition = (verticalEncoderRight.getCurrentPosition() * verticalRightEncoderPositionMultiplier);

            double leftChange = verticalLeftEncoderWheelPosition - previousVerticalLeftEncoderWheelPosition;
            double rightChange = verticalRightEncoderWheelPosition - previousVerticalRightEncoderWheelPosition;

            //Calculate Angle
            double changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
            robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));

            //Get the components of the motion
            double normalEncoderWheelPosition = (horizontalEncoder.getCurrentPosition() * normalEncoderPositionMultiplier);
            double rawHorizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;
            double horizontalChange = rawHorizontalChange - (changeInRobotOrientation * horizontalEncoderTickPerDegreeOffset);

            double p = ((rightChange + leftChange) / 2);

            //Calculate and update the position values
            robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (p * Math.sin(robotOrientationRadians) + horizontalChange * Math.cos(robotOrientationRadians));
            robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (p * Math.cos(robotOrientationRadians) - horizontalChange * Math.sin(robotOrientationRadians));

            previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
            previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
            prevNormalEncoderWheelPosition = normalEncoderWheelPosition;
        }
    }


    /**
     * Returns the robot's global orientation
     *
     * @return global orientation, in degrees
     */
    public double getRot() {
        return Math.toDegrees(robotOrientationRadians) % 360;
    }

    /**
     * Returns the robot's global X Position in inches
     *
     * @return global X Position in inches
     */
    public double getX() {

        return robotGlobalXCoordinatePosition / COUNTS_PER_INCH;
    }

    /**
     * Returns the robot's global Y Position in inches
     *
     * @return global Y Position in inches
     */
    public double getY() {

        return robotGlobalYCoordinatePosition / COUNTS_PER_INCH;
    }


    /**
     * Reverses the left encoder
     */
    public void reverseLeftEncoder() {
        if (verticalLeftEncoderPositionMultiplier == 1) {
            verticalLeftEncoderPositionMultiplier = -1;
        } else {
            verticalLeftEncoderPositionMultiplier = 1;
        }
    }

    /**
     * Reverses the right encoder
     */
    public void reverseRightEncoder() {
        if (verticalRightEncoderPositionMultiplier == 1) {
            verticalRightEncoderPositionMultiplier = -1;
        } else {
            verticalRightEncoderPositionMultiplier = 1;
        }
    }

    /**
     * Reverses the Horizontal encoder
     */
    public void reverseHorizontalEncoder() {
        if (normalEncoderPositionMultiplier == 1) {
            normalEncoderPositionMultiplier = -1;
        } else {
            normalEncoderPositionMultiplier = 1;
        }
    }

    @Override
    protected void onEnd() {
    }
}