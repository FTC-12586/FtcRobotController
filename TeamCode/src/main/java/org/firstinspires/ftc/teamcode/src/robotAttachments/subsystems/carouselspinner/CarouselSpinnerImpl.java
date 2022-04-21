package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.carouselspinner;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A class to control the robot carousel spinner
 */
public class CarouselSpinnerImpl implements CarouselSpinner {
    /**
     * How Fast the servo is to spin
     */
    private static final double servoPower = 1;

    /**
     * Multiplier for the right servo
     */
    private static final double rightServoMultiplier = 1;

    /**
     * Multiplier for the left servo
     */
    private static final double leftServoMultiplier = 1;

    /**
     * How long it takes to spin off a duck
     */
    private static final long duckSleepTime = 3100;

    /**
     * The left continuous servo
     */
    protected final CRServo leftServo;

    /**
     * The right continuous servo
     */
    protected final CRServo rightServo;

    /**
     * What percent of duckSleepTime the thread is to sleep for
     */
    private final double sleepPercentage = .90;

    /**
     * A constructor that sets up servo from Hardware map
     *
     * @param hardwareMap Hardware Map Object
     * @param spinnerOne  Name of continuous servo
     */
    public CarouselSpinnerImpl(HardwareMap hardwareMap, String spinnerOne, String spinnerTwo) {
        leftServo = hardwareMap.crservo.get(spinnerOne);
        rightServo = hardwareMap.crservo.get(spinnerTwo);
    }

    /**
     * Spins off the red duck
     *
     * @throws InterruptedException raises exception if the OpMode is stopped
     */
    public void spinOffRedDuck() throws InterruptedException {
        setServoPower(-servoPower);
        ElapsedTime t = new ElapsedTime();

        Thread.sleep((long) (duckSleepTime * sleepPercentage)); //Sleep for the majority of the duck spin time

        //Spin wait for the rest
        while (t.milliseconds() < duckSleepTime) {
            Thread.yield();
            if (Thread.currentThread().isInterrupted()) {
                throw new InterruptedException();
            }
        }
        this.halt();

    }

    /**
     * Spins off the blue duck
     *
     * @throws InterruptedException raises exception if the OpMode is stopped
     */
    public void spinOffBlueDuck() throws InterruptedException {
        setServoPower(servoPower);
        ElapsedTime t = new ElapsedTime();
        Thread.sleep((long) (duckSleepTime * sleepPercentage)); //Sleep for the majority of the duck spin time

        //Spin wait for the rest
        while (t.milliseconds() < duckSleepTime) {
            Thread.yield();
            if (Thread.currentThread().isInterrupted()) {
                throw new InterruptedException();
            }
        }
        this.halt();
    }

    @Override
    public void spinOffRedDuckSlow() throws InterruptedException {
        setServoPower(-0.5);

        ElapsedTime t = new ElapsedTime();

        Thread.sleep((long) (duckSleepTime * sleepPercentage)); //Sleep for the majority of the second half of duck spin time

        //Spin wait for the rest
        while (t.milliseconds() < duckSleepTime) {
            Thread.yield();
            if (Thread.currentThread().isInterrupted()) {
                throw new InterruptedException();
            }
        }
        this.halt();


    }

    @Override
    public void spinOffBlueDuckSlow() throws InterruptedException {
        setServoPower(0.5);

        ElapsedTime t = new ElapsedTime();
        Thread.sleep((long) (duckSleepTime * sleepPercentage)); //Sleep for the majority of first half of the duck spin time

        //Spin wait for the rest
        while (t.milliseconds() < duckSleepTime) {
            Thread.yield();
            if (Thread.currentThread().isInterrupted()) {
                throw new InterruptedException();
            }
        }
        this.halt();

    }

    /**
     * Sets power to the internal servos
     *
     * @param power The power to set the servos, in range [-1,1]
     */
    private void setServoPower(double power) {
        leftServo.setPower(power);
        rightServo.setPower(power);
    }

    /**
     * Stops the spinner
     */
    public void halt() {
        this.setServoPower(0);
    }

    /**
     * Allows control of a carousel spinner
     *
     * @param gamepad1 The first gamepad
     * @param gamepad2 The second gamepad
     * @return Always returns null
     */
    @Override
    public Void gamepadControl(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) {
        if (gamepad2.x) {
            this.setServoPower(servoPower);
        } else if (gamepad2.b) {
            this.setServoPower(-servoPower);
        } else {
            this.halt();
        }
        return null;
    }
}
