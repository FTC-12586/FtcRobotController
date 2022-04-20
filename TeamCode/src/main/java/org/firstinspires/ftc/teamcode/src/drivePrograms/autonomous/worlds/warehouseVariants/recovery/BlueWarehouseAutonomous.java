package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.warehouseVariants.recovery;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.WorldsAutonomousProgram;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutonomousTemplate;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//@Config
@Autonomous(name = "☠🟦Blue Warehouse Autonomous🟦☠", group = "recovery")
public class BlueWarehouseAutonomous extends WorldsAutonomousProgram {
    final static Pose2d startPos = new Pose2d(7, 65, 0);
    final static Pose2d dropOffPos = new Pose2d(-12, 38, Math.toRadians(90));
    final static Pose2d whEntryPos = new Pose2d(20, 65, Math.toRadians(0));

    public BlueWarehouseAutonomous() {
        super(GenericOpModeTemplate.LEDErrorColor);
    }

    public static TrajectorySequence toHub(SampleMecanumDrive drive, Pose2d startPos) {
        return drive.trajectorySequenceBuilder(startPos)
                .strafeRight(5)
                .lineToSplineHeading(dropOffPos.plus(new Pose2d(2, 3, 0)))
                //This path should not be replicated for additional freight since it is inefficient.
                .waitSeconds(1)
                .build();
    }

    public static TrajectorySequence hubToWarehouse(SampleMecanumDrive drive, Pose2d startPos) {
        return drive.trajectorySequenceBuilder(startPos)
                .splineToSplineHeading(whEntryPos.plus(new Pose2d(0, 0, 0)), Math.toRadians(0))

                .forward(20)
                .build();
    }

    @Override
    public void opModeMain() throws InterruptedException {
        ((AutonomousTemplate) this).initAll();

        final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPos);

        final TrajectorySequence startToHub = toHub(drive, startPos);

        final TrajectorySequence hubToWH = hubToWarehouse(drive, startToHub.end());

        telemetry.addData("Setup", "Finished");
        telemetry.update();

        final BarcodePositions pos = BarcodePositions.NotSeen;
        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(startToHub);

            this.dropOffItem(pos);

            drive.followTrajectorySequence(hubToWH);
            slide.setTargetLevel(HeightLevel.Down);
            slide.waitOn();

        }

    }
}
