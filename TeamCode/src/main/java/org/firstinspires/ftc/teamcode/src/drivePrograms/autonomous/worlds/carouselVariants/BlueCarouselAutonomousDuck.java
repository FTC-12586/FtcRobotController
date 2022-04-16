package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.carouselVariants;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.WorldsAutonomousProgram;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutonomousTemplate;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "🟦Blue Carousel Autonomous Duck🟦", group = "BlueCarousel")
public class BlueCarouselAutonomousDuck extends WorldsAutonomousProgram {
    final static Pose2d startPos = new Pose2d(-34, 65, 0);
    final static Pose2d dropOffPos = new Pose2d(-27, 23.5, Math.toRadians(180));
    final static Pose2d carouselSpinPos = new Pose2d(-61, 51, Math.toRadians(90));
    final static Pose2d duckPickupStart = new Pose2d(-45, 54, Math.toRadians(135));

    final static Pose2d parkPos = new Pose2d(-60, 35.5, Math.toRadians(90));

    public BlueCarouselAutonomousDuck() {
        super(BlinkinPattern.BLUE);
    }


    public static TrajectorySequence PickingUpDuck(SampleMecanumDrive drive, Pose2d startPos) {
        return drive.trajectorySequenceBuilder(startPos)
                .setConstraints((v, pose2d, pose2d1, pose2d2) -> 10,
                        (v, pose2d, pose2d1, pose2d2) -> 10)
                .lineTo(startPos.vec().minus(new Vector2d(0, 2)))
                .setReversed(true)
                .splineToLinearHeading(carouselSpinPos.plus(new Pose2d(14, 9, Math.toRadians(0))), Math.toRadians(90))
                .setReversed(false)
                .splineToLinearHeading(carouselSpinPos.plus(new Pose2d(25, 10, Math.toRadians(-45))), -45)

                .build();

    }

    public static TrajectorySequence CycleDuckGrab(SampleMecanumDrive drive, Pose2d startPos) {
        return drive.trajectorySequenceBuilder(startPos)
                .lineToConstantHeading(startPos.plus(new Pose2d(10, 12)).vec())
                .turn(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-37, 61, Math.toRadians(135)), Math.toRadians(135))
                .lineToConstantHeading(new Pose2d(-48, 61).vec())
                .build();
    }


    @Override
    public void opModeMain() throws InterruptedException {
        ((AutonomousTemplate) this).initAll();

        // From
        final Trajectory toGoal = BlueCarouselAutonomous.ToGoalTraj(drive, startPos);

        final TrajectorySequence toSpinner = BlueCarouselAutonomous.ToSpinner(drive, toGoal.end());

        final TrajectorySequence pickingUpDuck = PickingUpDuck(drive, toSpinner.end());

        final TrajectorySequence cycleDuckGrab = CycleDuckGrab(drive, pickingUpDuck.end());

        final Trajectory toPark = BlueCarouselAutonomous.ToEnd(drive, toSpinner.end());

        telemetry.addData("Setup", "Finished");
        telemetry.addData("end:", pickingUpDuck.end());
        telemetry.addData("Start", pickingUpDuck.start());
        telemetry.update();


        //final BarcodePositions pos = monitorMarkerWhileWaitForStart();
        waitForStart();

        this.closeWebcam();
        drive.setPoseEstimate(pickingUpDuck.start());


        if (!isStopRequested()) {
            intake.setMotorPower(1);
            checkStop();
            drive.followTrajectorySequence(pickingUpDuck);
            checkStop();
            drive.followTrajectorySequence(cycleDuckGrab);
            intake.setMotorPower(0);






/*
            drive.followTrajectory(toGoal);

            this.checkStop();

            drive.turnTo(dropOffPos.getHeading());

            this.checkStop();

            this.dropOffItem(pos);

            this.checkStop();

            drive.followTrajectorySequence(toSpinner);
            this.checkStop();

            slide.setTargetLevel(HeightLevel.Down);
            this.checkStop();

            spinner.spinOffBlueDuck();
            this.checkStop();

            //intake.setMotorPower(1);
            //this.checkStop();

            intake.setMotorPower(1);
            drive.followTrajectorySequence(pickingUpDuck);
            this.checkStop();

            drive.followTrajectorySequence(cycleDuckGrab);
            this.checkStop();
            intake.setMotorPower(0);





            //drive.strafeAtAngleWhileTurn(135,);


            drive.followTrajectory(toPark);
            this.checkStop();

 */


        }

    }

}
