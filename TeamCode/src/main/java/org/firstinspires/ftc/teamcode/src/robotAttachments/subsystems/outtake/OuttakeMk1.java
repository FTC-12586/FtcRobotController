package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.outtake;


import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.utills.enums.FreightFrenzyGameObject;
import org.firstinspires.ftc.teamcode.src.utills.enums.RGBCameraColors;

/**
 * this is the class for our robot's intake subsystem, It is the flippy intake Eli Tried
 */
public class OuttakeMk1 implements Outtake {

    /**
     * The Position servo must be to release an item
     */
    protected static final double onehundredEightyPos = .45 + 0.35; // this position needs to be adjusted!

    /**
     * The Position servo must be to release an item
     */
    protected static final double onehundredThirtyFivePos = .45 + (0.35 / 2); // this position needs to be adjusted!

    /**
     * The Position servo must be to release an item
     */
    protected static final double ninetyPos = .5; // this position needs to be adjusted!

    /**
     * The Position servo must be to keep and item in the intake compartment
     */
    protected static final double closed = 0.1; // this position needs to be adjusted

    /**
     * The item color sensor
     */
    protected final ColorRangeSensor colorSensor;

    /**
     * The internal Servo Object
     */
    protected final Servo itemRelease;
    protected final ElapsedTime closeTimer = new ElapsedTime();
    /**
     * A boolean that tells if the servo is closed or opened
     */
    protected boolean isClosed;
    protected boolean y_depressed2 = true;
    protected boolean b_depressed = true;
    protected boolean a_depressed = true;

    /**
     * Initializes from hardware map and names
     *
     * @param hardwareMap          hardware map object
     * @param servoName            Name of lifting servo
     * @param colorSensor          name of the color sensor
     * @param sensorDetectionLight true if the light should be on, false if the light should be off
     */
    public OuttakeMk1(HardwareMap hardwareMap, String servoName, String colorSensor, boolean sensorDetectionLight) {
        this.colorSensor = hardwareMap.get(ColorRangeSensor.class, colorSensor);
        this.colorSensor.enableLed(sensorDetectionLight);

        this.itemRelease = hardwareMap.servo.get(servoName);
        this.close();
    }

    /**
     * Initializes from hardware map and names
     *
     * @param hardwareMap hardware map object
     * @param servoName   Name of lifting servo
     * @param colorSensor name of the color sensor
     */
    public OuttakeMk1(HardwareMap hardwareMap, String servoName, String colorSensor) {
        this.colorSensor = hardwareMap.get(ColorRangeSensor.class, colorSensor);
        this.colorSensor.enableLed(true);

        this.itemRelease = hardwareMap.servo.get(servoName);
        this.close();
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
     * uses the intake's servo hinge to put the intake in the up position
     */
    public void open() {
        itemRelease.setPosition(ninetyPos);
        isClosed = false;
    }

    /**
     * uses the intake's servo hinge to put the intake in the down position
     */
    public void close() {
        itemRelease.setPosition(closed);
        isClosed = true;
    }

    /**
     * {@inheritDoc}
     */
    public void goTo(double pos) {
        itemRelease.setPosition(pos);
        isClosed = false;
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
        return FreightFrenzyGameObject.getLEDColorFromItem(FreightFrenzyGameObject.identify(this.getRGB()));
    }

    /**
     * Controls the intake and outtake
     *
     * @param gamepad1 The first gamepad
     * @param gamepad2 The second gamepad
     * @return The current item in the intake
     */
    @Override
    public FreightFrenzyGameObject gamepadControl(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) {

        FreightFrenzyGameObject currentObject = FreightFrenzyGameObject.EMPTY;

        if (!gamepad2.y) {
            y_depressed2 = true;
        }
        if (gamepad2.y && y_depressed2) {
            y_depressed2 = false;
            if (this.isClosed()) {
                this.open();
                this.closeTimer.reset();
            } else {
                this.close();
            }
        }

        if (!gamepad2.b) {
            b_depressed = true;
        }
        if (gamepad2.b && b_depressed) {
            b_depressed = false;
            if (this.isClosed()) {
                this.goTo(OuttakeMk1.onehundredThirtyFivePos);
                this.closeTimer.reset();
            } else {
                this.close();
            }
        }

        if (!gamepad2.a) {
            a_depressed = true;
        }
        if (gamepad2.a && a_depressed) {
            a_depressed = false;
            if (this.isClosed()) {
                this.goTo(OuttakeMk1.onehundredEightyPos);
                this.closeTimer.reset();
            } else {
                this.close();
            }
        }


        if (this.closeTimer.seconds() > 1.25) {
            this.close();
            if (this.isClosed) {
                currentObject = this.identifyContents();
            }
        }
        return currentObject;

    }

    /**
     * {@inheritDoc}
     */
    public void halt() {

    }
}

