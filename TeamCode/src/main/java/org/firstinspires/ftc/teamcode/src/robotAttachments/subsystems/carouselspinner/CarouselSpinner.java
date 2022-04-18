package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.carouselspinner;

import org.firstinspires.ftc.teamcode.src.utills.Controllable;

public interface CarouselSpinner extends Controllable<Void> {

    /**
     * Spins off the red duck
     *
     * @throws InterruptedException raises exception if the OpMode is stopped
     */
    void spinOffRedDuck() throws InterruptedException;

    /**
     * Spins off the blue duck
     *
     * @throws InterruptedException raises exception if the OpMode is stopped
     */
    void spinOffBlueDuck() throws InterruptedException;

    /**
     * Spins off the red duck
     *
     * @throws InterruptedException raises exception if the OpMode is stopped
     */
    void spinOffRedDuckSlow() throws InterruptedException;

    /**
     * Spins off the blue duck
     *
     * @throws InterruptedException raises exception if the OpMode is stopped
     */
    void spinOffBlueDuckSlow() throws InterruptedException;


}
