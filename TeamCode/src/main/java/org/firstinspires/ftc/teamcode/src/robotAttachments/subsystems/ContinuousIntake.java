package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.utills.enums.FreightFrenzyGameObject;
import org.firstinspires.ftc.teamcode.src.utills.enums.RGBCameraColors;

/**
 * this is the class for our robot's intake subsystem
 */
public class ContinuousIntake {
    /**
     * The power for going forward
     */
    private final static double forwardPower = 1;
    /**
     * The Position servo must be to release an item
     */
    private static final double open = .4; // this position needs to be adjusted!
    /**
     * The Position servo must be to keep and item in the intake compartment
     */
    private static final double closed = .7; // this position needs to be adjusted

    private boolean isClosed;
    /**
     * The item color sensor
     */
    public final ColorRangeSensor colorSensor;
    /**
     * DcMotor Object
     */
    private final DcMotor intakeMotor;
    /**
     * The internal Servo Object
     */
    private final Servo itemRelease;

    /**
     * Initializes from hardware map and names
     *
     * @param hardwareMap          hardware map object
     * @param motorName            Name of intake motor
     * @param servoName            Name of lifting servo
     * @param colorSensor          name of the color sensor
     * @param sensorDetectionLight true if the light should be on, false if the light should be off
     */
    public ContinuousIntake(HardwareMap hardwareMap, String motorName, String servoName, String colorSensor, boolean sensorDetectionLight) {
        intakeMotor = hardwareMap.dcMotor.get(motorName);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        this.colorSensor = hardwareMap.get(ColorRangeSensor.class, colorSensor);
        this.colorSensor.enableLed(sensorDetectionLight);


        itemRelease = hardwareMap.servo.get(servoName);
        this.setServoClosed();
        isClosed = true;
    }

    /**
     * It treats color as 3D space and returns the distance between the two points
     *
     * @param sight  The first set of RGB values
     * @param object The second set of RGB values
     * @return The distance between the two, smaller is closer
     */
    public static double getDifferenceOfColor(double[] sight, double[] object) {
        double r = Math.abs(sight[0] - object[0]);
        double g = Math.abs(sight[1] - object[1]);
        double b = Math.abs(sight[2] - object[2]);
        // this calculates the 3D distance between colors
        return Math.sqrt(Math.pow(Math.sqrt(Math.pow(r, 2) + Math.pow(g, 2)), 2) + Math.pow(b, 2));
    }

    /**
     * Identifies the contents in the bucket
     *
     * @return The {@link FreightFrenzyGameObject} inside the bucket
     */
    public FreightFrenzyGameObject identifyContents() {
        return FreightFrenzyGameObject.identify(this.getRGB());
    }


    /**
     * A getter for the isClosed boolean
     *
     * @return Returns true if the grabber is closed, false if otherwise
     */
    public boolean isClosed() {
        return this.isClosed;
    }

    /**
     * this turns on the intake motor to intake freight
     */
    public void setIntakeOn() {
        intakeMotor.setPower(forwardPower);
    }

    /**
     * turns off the intake motor
     */
    public void setIntakeOff() {
        intakeMotor.setPower(0);
    }

    /**
     * reverses the intake motor to remove freight from the intake bucket
     */
    public void setIntakeReverse() {
        intakeMotor.setPower(-forwardPower);
    }

    /**
     * @param power a variable input for the power of the intake motor
     *              this sets the power of the intake motor to the power variable input
     */
    public void setMotorPower(double power) {
        intakeMotor.setPower(power);
    }

    /**
     * uses the intake's servo hinge to put the intake in the up position
     */
    public void setServoOpen() {
        itemRelease.setPosition(open);
        isClosed = false;
    }

    /**
     * uses the intake's servo hinge to put the intake in the down position
     */
    public void setServoClosed() {
        itemRelease.setPosition(closed);
        isClosed = true;
    }

    /**
     * this following method takes a parameter for the type of color and outputs the sensor's number for that color
     *
     * @param color the name of the color wanted
     * @return this returns a number of the value for the name of the wanted color
     */
    public int getColor(RGBCameraColors color) {
        switch (color) {
            case Red:
                return colorSensor.red();

            case Blue:
                return colorSensor.blue();

            case Green:
                return colorSensor.green();

            case Alpha:
                return colorSensor.alpha();
            default:
                return 0;
        }
    }

    /**
     * Returns what the Color Sensor Sees
     *
     * @return Returns values from 0 to 255 in the form of R,G,B
     */
    public double[] getRGB() {
        return new double[]{colorSensor.red(), colorSensor.green(), colorSensor.blue()};
    }

    /**
     * Gets how close the Object is to the sensor
     *
     * @return The distance in Inches
     */
    public double getSensorDistance() {
        return colorSensor.getDistance(DistanceUnit.CM);
    }

    /**
     * @return returns a true or false value of whether or not an item passes the intake distance sensor
     */
    public boolean itemInBucket() {
        return (this.getSensorDistance() < 9);
    }

    /**
     * Analyzes the content of the bucket to determine shape, returns the corresponding blink pattern
     *
     * @return Returns the blink pattern for the object in the bucket
     */
    public RevBlinkinLedDriver.BlinkinPattern getLEDPatternFromFreight() {
        return FreightFrenzyGameObject.RevColorOfObj.get(FreightFrenzyGameObject.identify(this.getRGB()));
    }

}
