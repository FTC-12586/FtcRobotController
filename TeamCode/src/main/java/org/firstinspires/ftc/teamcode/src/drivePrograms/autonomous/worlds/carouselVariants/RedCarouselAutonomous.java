package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.carouselVariants;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.WorldsAutonomousProgram;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.LinearSlide;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//@Config
@Autonomous(name = "🟥Red Carousel Autonomous🟥", group = "RedCarousel")
public class RedCarouselAutonomous extends WorldsAutonomousProgram {
    static final Pose2d startPos = new Pose2d(-40, -65, Math.toRadians(0));
    static final Pose2d dropOffPos = new Pose2d(-33, -25, Math.toRadians(180));
    static final Pose2d parkPos = new Pose2d(-60, -35.5, Math.toRadians(90));
    static final Pose2d carouselSpinPos = new Pose2d(-65, -54, Math.toRadians(270));

    BarcodePositions detectedPos;

    public RedCarouselAutonomous() {
        super(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    public static Trajectory ToGoalTraj(SampleMecanumDrive drive, Pose2d startPos, LinearSlide slide, Executable<BarcodePositions> getPos) {
        return drive.trajectoryBuilder(startPos)
                // Side in
                .lineToConstantHeading(parkPos.plus(new Pose2d(10, -15, 0)).vec())
                // Cross Box
                .splineToSplineHeading(new Pose2d(parkPos.getX() + 12, parkPos.getY() + 10, dropOffPos.getHeading() + Math.toRadians(10)), Math.toRadians(0))
                .addSpatialMarker(
                        new Pose2d(
                                parkPos.getX() + 12, parkPos.getY() + 10, dropOffPos.getHeading() + Math.toRadians(10)
                        )
                                .plus(
                                        dropOffPos.plus(new Pose2d(2, 2, Math.toRadians(0)))
                                ).vec(), () -> {
                            switch (getPos.call()) {
                                case Center:
                                    slide.setTargetLevel(HeightLevel.MiddleLevel);
                                    break;

                                case Left:
                                    slide.setTargetLevel(HeightLevel.BottomLevel);
                                    break;

                                default:
                                    slide.setTargetLevel(HeightLevel.TopLevel);
                            }

                        })

                //Approach Goal
                .splineToSplineHeading(dropOffPos.plus(new Pose2d(2, 2, Math.toRadians(0))), Math.toRadians(0))
                .build();
    }

    public static TrajectorySequence ToSpinner(SampleMecanumDrive drive, Pose2d startPos, LinearSlide slide) {
        return drive.trajectorySequenceBuilder(startPos)

                .addSpatialMarker(new Pose2d(parkPos.getX() + 20, parkPos.getY() + 15, Math.toRadians(265)).vec().plus(startPos.vec()).div(2), () -> slide.setTargetLevel(HeightLevel.Down))

                //Back away from goal
                // Cross Box
                .lineToConstantHeading(new Pose2d(parkPos.getX() + 10, parkPos.getY() + 10, Math.toRadians(265)).vec())

                // Cross Box to Carousel Spinner
                .splineToSplineHeading(carouselSpinPos.plus(new Pose2d(10, 10, Math.toRadians(-30))), Math.toRadians(-90))
                .splineToConstantHeading(carouselSpinPos.plus(new Pose2d(0, 1, Math.toRadians(-30))).vec(), Math.toRadians(-90))
                .build();
    }

    public static Trajectory ToEnd(SampleMecanumDrive drive, Pose2d startPos, LinearSlide slide) {
        return drive.trajectoryBuilder(startPos)
                //Park

                .addSpatialMarker(startPos.vec().plus(parkPos.vec().plus(new Vector2d(0, 1))).div(2), () -> slide.setTargetLevel(HeightLevel.Down))

                .lineTo(parkPos.vec().plus(new Vector2d(-2, 3)))
                .build();
    }

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll(GenericOpModeTemplate.LeftWebcamName);

        final Executable<BarcodePositions> getPos = () -> detectedPos;

        final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        drive.setPoseEstimate(startPos);

        // From
        final Trajectory toGoal = RedCarouselAutonomous.ToGoalTraj(drive, startPos, slide, getPos);

        final TrajectorySequence toSpinner = RedCarouselAutonomous.ToSpinner(drive, toGoal.end(), slide);

        final Trajectory toPark = RedCarouselAutonomous.ToEnd(drive, toSpinner.end(), slide);

        telemetry.addData("Setup", "Finished");
        telemetry.update();

        detectedPos = this.monitorMarkerWhileWaitForStart();

        if (!isStopRequested()) {

            drive.followTrajectory(toGoal);

            drive.turnTo(dropOffPos.getHeading());

            this.dropOffItem(detectedPos);

            drive.followTrajectorySequence(toSpinner);

            slide.setTargetLevel(HeightLevel.Down);

            spinner.spinOffRedDuck();

            drive.followTrajectory(toPark);


        }
    }

}
