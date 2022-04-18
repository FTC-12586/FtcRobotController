package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplateCV;

public abstract class WorldsAutonomousProgram extends AutoObjDetectionTemplateCV {

    protected final RevBlinkinLedDriver.BlinkinPattern defaultColor;

    protected WorldsAutonomousProgram(RevBlinkinLedDriver.BlinkinPattern defaultColor) {
        this.defaultColor = defaultColor;
    }

    @Override
    public void initAll() throws InterruptedException {
        super.initAll();
        leds.setPattern(defaultColor);
        System.gc();
    }

    protected BarcodePositions monitorMarkerWhileWaitForStart() {

        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        BarcodePositions pos;
        do {
            try {
                pos = this.findPositionOfMarker();
            } catch (NullPointerException e) {
                pos = BarcodePositions.NotSeen;
                break;
            }
            telemetry.addData("Pos", pos);
            telemetry.update();

        } while (!opModeIsActive() && !isStarted());
        waitForStart();
        leds.setPattern(defaultColor);
        return pos;
    }

    protected void dropOffItem(BarcodePositions pos) throws InterruptedException {
        switch (pos) {
            case NotSeen:
            case Right:
                slide.setTargetLevel(HeightLevel.TopLevel);
                break;

            case Center:
                slide.setTargetLevel(HeightLevel.MiddleLevel);
                break;

            case Left:
                slide.setTargetLevel(HeightLevel.BottomLevel);
                break;
        }

        //Wait for the slide to reach position
        slide.waitOn();

        outtake.open();
        Thread.sleep(500);
        outtake.close();
    }

    protected void driveOverBarriers() throws InterruptedException {
        podServos.raise();
        Thread.sleep(1000);
        drive.setMotorPowers(1, 1, 1, 1);
        Thread.sleep(1000);
        drive.setMotorPowers(0, 0, 0, 0);
    }

    // this is used to intake freight without roadrunner
    public void blueAutoIntakeFreight(long millis, Intake intake, double power) {

        double angleInc = 0;// this double was intended to be an angle increment for turning, currently not in use

        intake.setFrontMotorPower(1); // the front intake is turned on

        ElapsedTime t = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        // we do not need to check for color sensor bc back intake motors are never spun

        while (!spaceBar.isPressed()) {
            //reset the time at the beginning of this loop
            t.reset();

            drive.goForwardSimple(power);
            //this moves the robot forward to intake

            while (t.milliseconds() < millis) {

                if (spaceBar.isPressed()) {

                    //once the spacebar is triggered turn off the front intake wheels and stop the movement
                    drive.halt();
                    intake.setFrontMotorPower(0);
                    return;
                }

                Thread.yield();
            }

            //reset the timer for the next movement backwards so it moves the sam distance as the forward movement
            t.reset();

            while (t.milliseconds() < millis) {
                // move backwards
                drive.goBackwardsSimple(power);
            }
            //ideally this would be where you turn
            //angleInc = drive.getImuAngle()-10;


        }


    }
}




