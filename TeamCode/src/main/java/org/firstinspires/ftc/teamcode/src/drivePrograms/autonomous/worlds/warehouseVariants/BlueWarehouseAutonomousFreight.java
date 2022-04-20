package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.warehouseVariants;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.WorldsAutonomousProgram;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.LinearSlide;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
//@Config
@Autonomous(name = "🟦Blue Warehouse Autonomous Freight🟦", group = "BlueWarehouse")
public class BlueWarehouseAutonomousFreight extends WorldsAutonomousProgram {
    final static Pose2d startPos = new Pose2d(7, 65, 0);
    final static Pose2d dropOffPos = new Pose2d(-12, 38, Math.toRadians(90));
    final static Pose2d whEntryPos = new Pose2d(20, 65, Math.toRadians(0));
    final static Pose2d freightPickup = new Pose2d(45, 45, Math.toRadians(45));


    public BlueWarehouseAutonomousFreight() {
        super(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    public static TrajectorySequence ToWareHouseFreight(SampleMecanumDrive drive, Pose2d startPos, LinearSlide slide) {
        return drive.trajectorySequenceBuilder(startPos)
                .forward(2)
                .addDisplacementMarker(() -> {
                    slide.setTargetLevel(HeightLevel.Down);
                })

                .splineToSplineHeading(whEntryPos.plus(new Pose2d(0, 0, 0)), Math.toRadians(0))

                .forward(10)
                .build();
    }

    public static TrajectorySequence ToHubFreight(SampleMecanumDrive drive, Pose2d startPos) {
        return drive.trajectorySequenceBuilder(startPos)
                .strafeRight(5)
                .lineToSplineHeading(dropOffPos.plus(new Pose2d(2, 3, 0)))
                //This path should not be replicated for additional freight since it is inefficient.
                .waitSeconds(1)
                .build();
    }


    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();

        drive.setPoseEstimate(startPos);

        final TrajectorySequence startToHub = BlueWarehouseAutonomous.toHub(drive, startPos);

        final TrajectorySequence toWHFreight = ToWareHouseFreight(drive, startToHub.end(), slide);

        final TrajectorySequence toHubFreight = ToHubFreight(drive, toWHFreight.end().plus(new Pose2d(0, 1, 0)));

        telemetry.addData("Setup", "Finished");
        telemetry.update();

        final BarcodePositions pos = monitorMarkerWhileWaitForStart();


        waitForStart();
        if (!isStopRequested() && opModeIsActive()) {
            drive.setPoseEstimate(startPos);

            drive.followTrajectorySequence(startToHub);

            this.dropOffItem(pos);
            slide.setTargetLevel(HeightLevel.Down);

            drive.followTrajectorySequence(toWHFreight);
            slide.setTargetLevel(HeightLevel.Down);
            slide.waitOn();


            blueAutoIntakeFreight(1000, intake, .3);


            drive.goLeftSimple(.3);
            Thread.sleep(500);
            drive.halt();
            // we are against wall
            // use distance sensor


            // the following could probably be put in a method in WorldsAutonomous
            //reposition using wall and distance sensor

            double xTarget = toWHFreight.end().getX();
            double xPos = (70 - (frontDistanceSensor.getDistance(DistanceUnit.INCH) + 7.25));
            double distanceToXPos = xTarget - xPos;
            double power;

            while (distanceToXPos > 1 || distanceToXPos < -1 && !isStopRequested() && opModeIsActive()) {


                xPos = (70 - (frontDistanceSensor.getDistance(DistanceUnit.INCH) + 7.25));// this is the X value from distance sensor
                distanceToXPos = xTarget - xPos; // the distance between ideal and actual


                power = (.1 + distanceToXPos / 20);// power is proportional plus a baseline of .1

                if (distanceToXPos > 1) {
                    drive.goForwardSimple(power);
                    telemetry.addData("", intakeDistanceSensor.getDistance(DistanceUnit.INCH) + 7.25);
                    telemetry.update();
                } else {
                    drive.goBackwardsSimple(power);
                    telemetry.addData("", intakeDistanceSensor.getDistance(DistanceUnit.INCH) + 7.25);
                    telemetry.update();
                }
            }
            drive.halt();

            // from this point on we should have a consistent starting value compatible with a RR predetermined position

            drive.followTrajectorySequence(toHubFreight);













            /*
            drive.goLeftSimple(.3);
            Thread.sleep(500);

            */

        }

    }
}
