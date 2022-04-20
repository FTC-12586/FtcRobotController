package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.carouselVariants;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.WorldsAutonomousProgram;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.LinearSlide;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//@Config
@Autonomous(name = "🟦Blue Carousel Autonomous🟦", group = "BlueCarousel")
public class BlueCarouselAutonomous extends WorldsAutonomousProgram {
    final static Pose2d startPos = new Pose2d(-34, 65, 0);
    final static Pose2d dropOffPos = new Pose2d(-27, 23.5, Math.toRadians(180));
    final static Pose2d carouselSpinPos = new Pose2d(-61, 51, Math.toRadians(90));
    final static Pose2d parkPos = new Pose2d(-60, 35.5, Math.toRadians(90));

    BarcodePositions detectedPos;

    public BlueCarouselAutonomous() {
        super(BlinkinPattern.BLUE);
    }

    public static Trajectory ToGoalTraj(SampleMecanumDrive drive, Pose2d startPos, LinearSlide slide, Executable<BarcodePositions> getPos) {
        return drive.trajectoryBuilder(startPos)
                // Side in
                .lineToConstantHeading(parkPos.plus(new Pose2d(18, 15, 0)).vec())
                // Cross Box
                .splineToSplineHeading(new Pose2d(parkPos.getX() + 20, parkPos.getY() - 15, dropOffPos.getHeading()), Math.toRadians(0))
                .addSpatialMarker(dropOffPos.vec(), () -> {
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
                .splineToSplineHeading(dropOffPos.plus(new Pose2d(12, 4, Math.toRadians(20))), Math.toRadians(0))
                .build();
    }

    public static TrajectorySequence ToSpinner(SampleMecanumDrive drive, Pose2d startPos, LinearSlide slide) {
        return drive.trajectorySequenceBuilder(startPos)

                .addSpatialMarker(new Pose2d(parkPos.getX() + 20, parkPos.getY() - 15, Math.toRadians(95)).vec().plus(startPos.vec()).div(2), () -> slide.setTargetLevel(HeightLevel.Down))

                //Back away from goal
                // Cross Box
                .lineToConstantHeading(new Pose2d(parkPos.getX() + 20, parkPos.getY() - 15, Math.toRadians(95)).vec())

                // Cross Box to Carousel Spinner
                .splineToSplineHeading(carouselSpinPos.plus(new Pose2d(10, -10, Math.toRadians(30))), Math.toRadians(90))
                .splineToConstantHeading(carouselSpinPos.plus(new Pose2d(5, 3, Math.toRadians(30))).vec(), Math.toRadians(90))
                .build();
    }

    public static Trajectory ToEnd(SampleMecanumDrive drive, Pose2d startPos, LinearSlide slide) {
        return drive.trajectoryBuilder(startPos)
                //Park
                .addSpatialMarker(startPos.plus(parkPos.plus(new Pose2d(10, -1))).div(2).vec(), () -> slide.setTargetLevel(HeightLevel.Down))
                .lineTo(parkPos.vec().plus(new Vector2d(10, -1)))
                .build();
    }

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();

        leds.setPattern(defaultColor);

        drive.setPoseEstimate(startPos);

        final Executable<BarcodePositions> getPos = () -> detectedPos;

        // From
        final Trajectory toGoal = BlueCarouselAutonomous.ToGoalTraj(drive, startPos, slide, getPos);

        final TrajectorySequence toSpinner = BlueCarouselAutonomous.ToSpinner(drive, toGoal.end(), slide);

        final Trajectory toPark = ToEnd(drive, toSpinner.end(), slide);

        telemetry.addData("Setup", "Finished");
        telemetry.update();

        detectedPos = monitorMarkerWhileWaitForStart();


        waitForStart();

        if (!isStopRequested()) {

            drive.followTrajectory(toGoal);
            drive.turnTo(dropOffPos.getHeading());

            this.dropOffItem(detectedPos);

            drive.followTrajectorySequence(toSpinner);

            slide.setTargetLevel(HeightLevel.Down);

            spinner.spinOffBlueDuck();

            drive.followTrajectory(toPark);


        }
    }

}
