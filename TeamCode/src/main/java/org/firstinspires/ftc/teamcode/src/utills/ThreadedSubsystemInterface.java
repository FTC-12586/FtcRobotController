package org.firstinspires.ftc.teamcode.src.utills;

/**
 * This is a template for all subsystems that need threading, provides thread safety
 */
public interface ThreadedSubsystemInterface extends Runnable {

    /**
     * It is the main function of the thread
     */
    void threadMain();

    /**
     * Ends the life of this thread
     */
    void end();

    /**
     * Returns the running state of the thread
     *
     * @return true if running, false otherwise
     */
    boolean isRunning();


}