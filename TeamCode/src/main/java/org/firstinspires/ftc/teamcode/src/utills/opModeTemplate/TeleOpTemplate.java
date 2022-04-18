package org.firstinspires.ftc.teamcode.src.utills.opModeTemplate;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.robotAttachments.driveTrains.TeleopDriveTrain;


/**
 * A template for all Teleop opModes, allows easy initialization
 */
@Disabled
public abstract class TeleOpTemplate extends GenericOpModeTemplate {

    /**
     * Provides methods for driving
     */
    protected TeleopDriveTrain driveTrain;

    /**
     * Allows the control of the intake
     */

    public static final BlinkinPattern SpaceBarBlinkColor = BlinkinPattern.BLACK;


    /**
     * Initializes the objects provided by this class
     */
    protected void initAll() throws InterruptedException {
        initDriveTrain();

        initOuttake();

        super.initAll();
        podServos.raise();

        telemetry.addData("Initialization Status", "Initialized");
        telemetry.update();
    }


    /**
     * Initializes the Drive Train
     */
    protected void initDriveTrain() {
        driveTrain = new TeleopDriveTrain(hardwareMap, frontRightName, frontLeftName, backRightName, backLeftName, true);
    }


    /**
     * Initializes the Odometry Servos
     */
    protected void initOdometryServos() {
        super.initOdometryServos();
        podServos.raise();
    }


}
