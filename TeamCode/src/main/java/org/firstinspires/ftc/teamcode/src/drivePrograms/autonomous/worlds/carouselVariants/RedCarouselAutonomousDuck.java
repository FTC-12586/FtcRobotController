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
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//@Config
//@Disabled
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
                .back(2)
                .lineToConstantHeading(startPos.plus(new Pose2d(3, 0)).vec())
                .lineToConstantHeading(startPos.plus(new Pose2d(3, -3)).vec())
                .splineToConstantHeading(startPos.plus(new Pose2d(6, -6)).vec(), Math.toRadians(285))
                .splineToConstantHeading(startPos.plus(new Pose2d(8, -7)).vec(), Math.toRadians(285))
                .splineToConstantHeading(startPos.plus(new Pose2d(12, -8)).vec(), 0)

                .build();

    }

    public static TrajectorySequence BackToGoalTraj(SampleMecanumDrive drive, Pose2d startPos, LinearSlide slide, Executable<BarcodePositions> getPos) {
        return drive.trajectorySequenceBuilder(startPos)
                // Cross Box

                .setConstraints((v, pose2d, pose2d1, pose2d2) -> 15, (v, pose2d, pose2d1, pose2d2) -> 15)

                .splineToSplineHeading(new Pose2d(parkPos.getX(), parkPos.getY() + 5, dropOffPos.getHeading()), Math.toRadians(90))

                .addSpatialMarker(dropOffPos.vec().plus(new Vector2d(-15)), () -> {
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
                .splineToSplineHeading(dropOffPos.plus(new Pose2d(7, 7, Math.toRadians(5))), Math.toRadians(0))


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
        this.initAll(GenericOpModeTemplate.LeftWebcamName);

        drive.setPoseEstimate(startPos);

        // From
        final TrajectorySequence toGoal = RedCarouselAutonomous.ToGoalTraj(drive, startPos, slide, getPos);
        checkStop();

        final TrajectorySequence toSpinner = RedCarouselAutonomous.ToSpinner(drive, toGoal.end(), slide);
        checkStop();

        final TrajectorySequence pickingUpDuck = PickingUpDuck(drive, toSpinner.end());

        checkStop();

        final TrajectorySequence backToGoal = BackToGoalTraj(drive, pickingUpDuck.end(), slide, getPos);
        checkStop();

        Pose2d startPos1 = backToGoal.end();
        final Trajectory toPark = drive.trajectoryBuilder(startPos1)
                .forward(7)

                //Park
                .addSpatialMarker(startPos1.vec().plus(RedCarouselAutonomous.parkPos.vec().plus(new Vector2d(0, 1))).div(2), () -> slide.setTargetLevel(HeightLevel.Down))

                .splineTo(RedCarouselAutonomous.parkPos.vec().plus(new Vector2d(-2, 3)), Math.toRadians(270))
                .build();
        checkStop();

        final Trajectory altPark = AltPark(drive, pickingUpDuck.end(), slide);
        checkStop();

        telemetry.addData("Setup", "Finished");
        telemetry.update();

        detectedPos = monitorMarkerWhileWaitForStart();

        waitForStart();

        this.closeWebcam();

        drive.setPoseEstimate(startPos);

        if (!isStopRequested() && opModeIsActive()) {

            drive.followTrajectorySequence(toGoal);

            drive.turnTo(dropOffPos.getHeading());

            this.dropOffItem(detectedPos);

            drive.followTrajectorySequence(toSpinner);

            slide.setTargetLevel(HeightLevel.Down);

            spinner.spinOffRedDuck();

            intake.setMotorPower(1);

            drive.followTrajectorySequence(pickingUpDuck);

            detectedPos = BarcodePositions.NotSeen;

            drive.followTrajectorySequence(backToGoal);

            outtake.open();

            intake.setMotorPower(0);

            this.dropOffItem(BarcodePositions.NotSeen);

            drive.followTrajectory(toPark);


            if (true) {//outtake.identifyContents() != FreightFrenzyGameObject.EMPTY) {

                //detectedPos = BarcodePositions.NotSeen;

                //drive.followTrajectorySequence(backToGoal);

                //intake.setMotorPower(0);

                //this.dropOffItem(BarcodePositions.NotSeen);

                //drive.followTrajectory(toPark);
            } else {
                drive.followTrajectory(altPark);
            }


        }

    }

}
