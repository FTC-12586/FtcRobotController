package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.warehouseVariants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.WorldsAutonomousProgram;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "🟦Blue Warehouse Autonomous Freight🟦", group = "BlueWarehouse")
public class BlueWarehouseAutonomousFreight extends WorldsAutonomousProgram {
    final static Pose2d startPos = new Pose2d(7, 65, 0);
    final static Pose2d dropOffPos = new Pose2d(-12, 38, Math.toRadians(90));
    final static Pose2d whEntryPos = new Pose2d(20, 65, Math.toRadians(0));
    final static Pose2d freightPickup = new Pose2d(45, 45, Math.toRadians(45));

    public BlueWarehouseAutonomousFreight() {
        super(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    public static TrajectorySequence ToWareHouseFreight(SampleMecanumDrive drive, Pose2d startPos) {
        return drive.trajectorySequenceBuilder(startPos)
                .splineToSplineHeading(whEntryPos.plus(new Pose2d(0, 0, 0)), Math.toRadians(0))

                .forward(10)

                .splineToSplineHeading(freightPickup, 0)
                .setReversed(true)
                .splineToSplineHeading(whEntryPos.plus(new Pose2d(10, 0, 0)), Math.toRadians(180))
                .back(10)

                .splineToSplineHeading(dropOffPos, Math.toRadians(-90))
                .setReversed(false)
                .build();
    }

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();

        final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPos);

        final TrajectorySequence startToHub = BlueWarehouseAutonomous.toHub(drive, startPos);

        final TrajectorySequence toWHFreight = ToWareHouseFreight(drive, startToHub.end());

        telemetry.addData("Setup", "Finished");
        telemetry.update();

        final BarcodePositions pos = monitorMarkerWhileWaitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(startToHub);

            this.dropOffItem(pos);

            drive.followTrajectorySequence(toWHFreight);
            slide.setTargetLevel(HeightLevel.Down);
            slide.waitOn();

        }

    }
}
