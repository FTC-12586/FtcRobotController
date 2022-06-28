package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide;

import org.firstinspires.ftc.teamcode.src.utills.Controllable;

/**
 * An interface for all Linear Slides to follow
 */
public interface LinearSlide extends Controllable<Void> {
    /**
     * Sets the linear slide to go to the provided position
     *
     * @param targetPosition The position to go to in ticks
     */
    void setTargetPosition(int targetPosition);

    /**
     * Sets the slide to go to the provided level
     *
     * @param level The level to go to
     */
    void setTargetLevel(HeightLevel level);

    /**
     * Waits for the slide to get position
     * @throws InterruptedException If the OpMode is stopped while waiting
     */
    void waitOn() throws InterruptedException;

    /**
     *
     * @return True if the slide is at it's targeted position
     */
    boolean isAtPosition();

    /**
     * Returns true if the slide is close to it's set position
     *
     * @param tolerance The maximum distance away the slide can be and still return true
     * @return true if the slide is close, false if it is not
     */
    boolean isAtPosition(int tolerance);

    /**
     * A getter for the position the slide is currently at
     *
     * @return The position of the slide in ticks
     */
    int getEncoderCount();

    /**
     * Resets the encoders, reverts to autonomous mode
     */
    void resetEncoder();

    /**
     * Changes the linear slide to work by setting power
     */
    void teleopMode();

    /**
     * Changes the linear slide to work by going to position
     */
    void autoMode();

    /**
     * Reverses the motor
     */
    void reverseMotor();

    /**
     * Sets the power to the motor
     *
     * @param power The power to set to between the range of -1 and 1
     */
    void setMotorPower(double power);

    /**
     * Getter for the target height
     *
     * @return Returns the position the Slide is set to go to
     */
    int getTargetHeight();

}
