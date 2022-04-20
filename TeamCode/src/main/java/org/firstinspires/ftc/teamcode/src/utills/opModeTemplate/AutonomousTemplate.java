package org.firstinspires.ftc.teamcode.src.utills.opModeTemplate;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;

/**
 * A template for Autonomous OpModes, allows for easy initialization
 */
@Disabled
public abstract class AutonomousTemplate extends GenericOpModeTemplate {

    protected SampleMecanumDrive drive;


    /**
     * Initializes all fields provided by this class
     *
     * @throws InterruptedException Throws if the OpMode is stopped during function execution
     */
    public void initAll() throws InterruptedException {
        RobotLog.d("Robot Init Started");

        this.initOdometryServos();
        podServos.raise();
        super.initAll();
        podServos.lower();
        slide.setTargetLevel(HeightLevel.Down);
        drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addData("Default Initialization: ", "Finished");
        telemetry.update();
        checkStop();

        RobotLog.d("Robot Init Finished");
    }

    @Override
    protected void initLinearSlide() throws InterruptedException {
        super.initLinearSlide();
        slide.autoMode();
    }

}
