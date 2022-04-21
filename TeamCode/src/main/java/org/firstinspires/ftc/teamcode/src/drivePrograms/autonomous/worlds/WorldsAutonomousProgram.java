package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtils;
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
        checkStop();
        System.gc();
        checkStop();
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

    /**
     * @param initialDistance this is the initial distance from the desired point of the movement
     * @param currentDistance this is the current distance from the desired location
     * @param maximumPower    this is the maximum power of the movement
     * @param minimumPower    this is the minimum power of the movement
     * @return this function returns a double power variable that is scaled to the down slope of a sin wave
     * this method has to be within a loop to iterate and return scaled power values
     */
    // this is a method from state that could possibly be used in some of our non- RoadRunner movements
    protected double shortMovementPowerCalculation(final double initialDistance, double currentDistance, double maximumPower, double minimumPower) {
        double powerOutput = MiscUtils.boundNumber(Math.sin((Math.PI * currentDistance) / (2 * initialDistance)));
        return MiscUtils.map(powerOutput, 0, 1, minimumPower, maximumPower);
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
        double tmpMillis = millis;
        boolean holding = false;

        intake.setFrontMotorPower(1); // the front intake is turned on

        ElapsedTime t = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        // we do not need to check for color sensor bc back intake motors are never spun

        while (!spaceBar.isPressed() || !holding) {
            //reset the time at the beginning of this loop
            t.reset();

            //this moves the robot forward to intake
            drive.goForwardSimple(power);

            while (t.milliseconds() < millis) {

                if (spaceBar.isPressed()) {

                    //once the spacebar is triggered turn off the front intake wheels and stop the movement

                    //stop the front intake
                    intake.setFrontMotorPower(0);

                    // we are holding something
                    holding = true;

                    //this is how many millis we have moved forward and how many millis we should move back
                    tmpMillis = t.milliseconds();

                    //stop driving forward
                    drive.halt();

                    //break out of this timing loop
                    break;

                    //return;
                }

                Thread.yield();
            }

            //reset the timer for the next movement backwards so it moves the sam distance as the forward movement
            t.reset();

            while (t.milliseconds() < tmpMillis) {

                // move backwards same tmpMillis as we did forward
                drive.goBackwardsSimple(power);
            }

            // stop the robot
            drive.halt();


            if (!holding) {
                // if we did not pick anything up we turn to move forward again


                //ideally this would be where you turn
                //angleInc = drive.getImuAngle()-10;
            }


        }


    }

    /**
     * @param roadRunnerXVal the .end().getX() of the previous roadrunner movement
     * @param tolerance      The tolerance for how close it needs to get
     */
    public final void distanceSensorReposition(final double roadRunnerXVal, final double tolerance) {
        double xPos = (70 - (frontDistanceSensor.getDistance(DistanceUnit.INCH) + 7.25));

        double distanceToXPos = roadRunnerXVal - xPos;// this will be negative when robot is further forward and positive when robot is too far back
        double initialDistance = distanceToXPos;

        while (Math.abs(distanceToXPos) > tolerance && !isStopRequested() && opModeIsActive()) {

            //70 is the rr distance scaling variable
            final int ROADRUNNER_SCALE_VAL = 70;

            //7.25 is the distance the center of the robot is from the distance sensor
            final double DISTANCE_SENSOR_OFFSET = 7.25;


            xPos = (ROADRUNNER_SCALE_VAL - (frontDistanceSensor.getDistance(DistanceUnit.INCH) + DISTANCE_SENSOR_OFFSET));// this is the X value from distance sensor
            distanceToXPos = roadRunnerXVal - xPos; // the distance between ideal and actual

            // final double power = (.1 + distanceToXPos / 20);// power is proportional plus a baseline of .1

            final double power = shortMovementPowerCalculation(initialDistance, distanceToXPos, .3, .1);


            if (Math.abs(distanceToXPos) > tolerance) {
                drive.goForwardSimple(power);
                telemetry.addData("", frontDistanceSensor.getDistance(DistanceUnit.INCH) + 7.25);
                telemetry.update();
            }

        }
        drive.halt();
    }

}




