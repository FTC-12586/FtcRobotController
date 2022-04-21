package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.carouselVariants;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.WorldsAutonomousProgram;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.LinearSlide;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//@Config
@Disabled
@Autonomous(name = "🟥Red Carousel Autonomous Duck🟥", group = "RedCarousel")
public class RedCarouselAutonomousDuck extends WorldsAutonomousProgram {
    static final Pose2d startPos = new Pose2d(-40, -65, Math.toRadians(0));
    static final Pose2d dropOffPos = new Pose2d(-33, -25, Math.toRadians(180));
    static final Pose2d parkPos = new Pose2d(-60, -35.5, Math.toRadians(90));
    static final Pose2d carouselSpinPos = new Pose2d(-65, -54, Math.toRadians(270));

    BarcodePositions detectedPos;

    public RedCarouselAutonomousDuck() {
        super(BlinkinPattern.RED);
    }


    public static TrajectorySequence PickingUpDuck(SampleMecanumDrive drive, Pose2d startPos) {
        return drive.trajectorySequenceBuilder(startPos)
                .setConstraints((these, are, velocity, parameters) -> 5, (these, are, acceleration, parameters) -> 15)

                .lineToConstantHeading(startPos.plus(new Pose2d(6, 0)).vec())
                .splineToConstantHeading(startPos.plus(new Pose2d(6, -6)).vec(), Math.toRadians(285))
                .splineToConstantHeading(startPos.plus(new Pose2d(8, -8)).vec(), Math.toRadians(285))
                .splineToConstantHeading(startPos.plus(new Pose2d(12, -12)).vec(), 0)

                .build();

    }

    public static Trajectory BackToGoalTraj(SampleMecanumDrive drive, Pose2d startPos, LinearSlide slide, Executable<BarcodePositions> getPos) {
        return drive.trajectoryBuilder(startPos)
                // Cross Box
                .splineToSplineHeading(new Pose2d(parkPos.getX() + 20, parkPos.getY() + 15, dropOffPos.getHeading()), Math.toRadians(180))
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
                .splineToSplineHeading(dropOffPos.plus(new Pose2d(12, -8, Math.toRadians(-20))), Math.toRadians(0))
                .build();
    }

    public static Trajectory AltPark(SampleMecanumDrive drive, Pose2d startPos, LinearSlide slide) {
        return drive.trajectoryBuilder(startPos)

                .addSpatialMarker(startPos.vec().plus(parkPos.vec().plus(new Vector2d(10, 1))).div(2), () -> slide.setTargetLevel(HeightLevel.Down))

                //Park
                .lineTo(parkPos.vec().plus(new Vector2d(10, 1)))
                .build();
    }


    @Override
    public void opModeMain() throws InterruptedException {
        final Executable<BarcodePositions> getPos = () -> detectedPos;
        this.initAll();

        drive.setPoseEstimate(startPos);

        // From
        final Trajectory toGoal = RedCarouselAutonomous.ToGoalTraj(drive, startPos, slide, getPos);

        final TrajectorySequence toSpinner = RedCarouselAutonomous.ToSpinner(drive, toGoal.end(), slide);

        final TrajectorySequence pickingUpDuck = PickingUpDuck(drive, toSpinner.end());

        final Trajectory backToGoal = BackToGoalTraj(drive, pickingUpDuck.end(), slide, getPos);

        final Trajectory toPark = RedCarouselAutonomous.ToEnd(drive, backToGoal.end(), slide);

        final Trajectory altPark = AltPark(drive, pickingUpDuck.end(), slide);

        telemetry.addData("Setup", "Finished");
        telemetry.update();

        detectedPos = monitorMarkerWhileWaitForStart();

        waitForStart();

        this.closeWebcam();

        drive.setPoseEstimate(startPos);

        if (!isStopRequested() && opModeIsActive()) {

            drive.followTrajectory(toGoal);

            drive.turnTo(dropOffPos.getHeading());

            this.dropOffItem(detectedPos);

            drive.followTrajectorySequence(toSpinner);

            slide.setTargetLevel(HeightLevel.Down);

            spinner.spinOffRedDuckSlow();

            intake.setMotorPower(1);

            drive.followTrajectorySequence(pickingUpDuck);

            if (true) {//outtake.identifyContents() != FreightFrenzyGameObject.EMPTY) {

                detectedPos = BarcodePositions.NotSeen;

                drive.followTrajectory(backToGoal);

                intake.setMotorPower(0);

                this.dropOffItem(BarcodePositions.NotSeen);

                drive.followTrajectory(toPark);
            } else {
                drive.followTrajectory(altPark);
            }
        }

    }

}