package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.carouselVariants;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.WorldsAutonomousProgram;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@SuppressWarnings("unused")
//@Config
@Disabled
@Deprecated
@Autonomous(name = "🟥Red Carousel WH Park Autonomous V1🟥", group = "RedCarousel")
public class RedCarouselAutonomousWHParkV1 extends WorldsAutonomousProgram {
    static final Pose2d startPos = new Pose2d(-40, -65, Math.toRadians(0));
    static final Pose2d dropOffPos = new Pose2d(-33, -25, Math.toRadians(180));
    static final Pose2d parkPos = new Pose2d(-60, -35.5, Math.toRadians(270));
    static final Pose2d carouselSpinPos = new Pose2d(-65, -54, Math.toRadians(270));
    final static Pose2d warehouseCrossPos = new Pose2d(11, -46, 0);

    public RedCarouselAutonomousWHParkV1() {
        super(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    public static TrajectorySequence toEnd(SampleMecanumDrive drive, Pose2d startPos) {
        return drive.trajectorySequenceBuilder(startPos)
                //Park
                .back(2)
                .lineToLinearHeading(warehouseCrossPos.plus(new Pose2d(5, -20, Math.toRadians(-20))))
                .build();
    }

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll(LeftWebcamName);

        final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        drive.setPoseEstimate(startPos);

        // From
        final Trajectory toGoal = RedCarouselAutonomousV1.ToGoalTraj(drive, startPos);

        final TrajectorySequence toSpinner = RedCarouselAutonomousV1.ToSpinner(drive, toGoal.end());

        final TrajectorySequence toPark = toEnd(drive, toSpinner.end());

        telemetry.addData("Setup", "Finished");
        telemetry.update();
        BarcodePositions pos = this.monitorMarkerWhileWaitForStart();


        if (!isStopRequested()) {

            drive.followTrajectory(toGoal);
            drive.turnTo(dropOffPos.getHeading());

            this.dropOffItem(pos);

            drive.followTrajectorySequence(toSpinner);
            slide.setTargetLevel(HeightLevel.Down);

            spinner.spinOffRedDuck();

            drive.followTrajectorySequence(toPark);

            this.driveOverBarriers();


        }
    }

}
