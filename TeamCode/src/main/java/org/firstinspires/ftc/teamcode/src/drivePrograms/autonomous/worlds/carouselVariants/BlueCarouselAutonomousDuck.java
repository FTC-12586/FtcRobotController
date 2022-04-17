package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.carouselVariants;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.WorldsAutonomousProgram;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutonomousTemplate;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

@Config
@Autonomous(name = "🟦Blue Carousel Autonomous Duck🟦", group = "BlueCarousel")
public class BlueCarouselAutonomousDuck extends WorldsAutonomousProgram {
    final static Pose2d startPos = BlueCarouselAutonomous.startPos;
    final static Pose2d dropOffPos = BlueCarouselAutonomous.dropOffPos;
    final static Pose2d carouselSpinPos = BlueCarouselAutonomous.carouselSpinPos;
    final static Pose2d duckPickupStart = new Pose2d(-45, 54, Math.toRadians(135));


    final static Pose2d parkPos = new Pose2d(-60, 35.5, Math.toRadians(90));
    final static Pose2d backUp = new Pose2d(-45, 52, Math.toRadians(45));
    final static Pose2d sweepPt1 = new Pose2d(-30, 55, Math.toRadians(90));
    final static Pose2d sweepPt2 = new Pose2d(-37, 61, Math.toRadians(135));
    final static Pose2d sweepPt3 = new Pose2d(-48, 61, Math.toRadians(135));
    final static double angle = Math.toRadians(10);

    public BlueCarouselAutonomousDuck() {
        super(BlinkinPattern.BLUE);
    }

    ArrayList<String> log = new ArrayList<String>();


    public static TrajectorySequence PickingUpDuck(SampleMecanumDrive drive, Pose2d startPos, ArrayList<String> log) {
        return drive.trajectorySequenceBuilder(startPos)
                .setConstraints((v, pose2d, pose2d1, pose2d2) -> 5,
                        (v, pose2d, pose2d1, pose2d2) -> 5)

                .waitSeconds(1)

                .lineTo(startPos.vec().minus(new Vector2d(0, 2)))

                .setReversed(true)
                .splineToLinearHeading(carouselSpinPos.plus(new Pose2d(14, 13, Math.toRadians(-20))), Math.toRadians(90))
                .setReversed(false)
                .splineToLinearHeading(carouselSpinPos.plus(new Pose2d(25, 13, Math.toRadians(-25))), -45)

                .build();

    }

    public static TrajectorySequence TryThis(SampleMecanumDrive drive, Pose2d startPos) {
        /*
        this was another pickup duck idea that was started
        but was too work intensive to get working soon enough
        the Meep Meep example/idea is in Trajectories Test
         */
        return drive.trajectorySequenceBuilder(startPos)
                .back(5)
                .turn(Math.toRadians(-20))

                .lineToConstantHeading(startPos.plus(new Pose2d(24, -3)).vec())
                .turn(Math.toRadians(25))
                .lineToConstantHeading(startPos.plus(new Pose2d(0, -1)).vec())
                .turn(Math.toRadians(-10))
                .lineToConstantHeading(startPos.plus(new Pose2d(14, 1)).vec())
                .turn(Math.toRadians(-45))
                .turn(Math.toRadians(70))
                .lineToConstantHeading(startPos.plus(new Pose2d(0, 3)).vec())
                .turn(Math.toRadians(-10))
                .lineToConstantHeading(startPos.plus(new Pose2d(24, 5)).vec())
                .turn(Math.toRadians(25))
                .lineToConstantHeading(startPos.plus(new Pose2d(0, 7)).vec())
                .turn(Math.toRadians(-10))
                .build();
    }

    public static TrajectorySequence CycleDuckGrab(SampleMecanumDrive drive, Pose2d startPos, ArrayList<String> log, Telemetry telemetry) {
        return drive.trajectorySequenceBuilder(startPos)
                .setConstraints((v, pose2d, pose2d1, pose2d2) -> 5,
                        (v, pose2d, pose2d1, pose2d2) -> 5)


                .setReversed(true)
                .splineToLinearHeading(backUp.plus(new Pose2d(5, 3)), Math.toRadians(225))
                //this spline moves backwards

                .setReversed(false)
                .turn(Math.toRadians(-60))
                //the robot turns

                .splineToLinearHeading(sweepPt2.plus(new Pose2d(3, 7, Math.toRadians(-15))), Math.toRadians(120))
                .lineToConstantHeading(sweepPt3.plus(new Pose2d(4, 7, 0)).vec())
                // the robot moves in a sweeping motion to pick up additional freight

                .build();
    }


    @Override
    public void opModeMain() throws InterruptedException {
        ((AutonomousTemplate) this).initAll();

        // From
        ((AutonomousTemplate) this).initAll();

        leds.setPattern(defaultColor);

        drive.setPoseEstimate(startPos);

        // From
        final Trajectory toGoal = BlueCarouselAutonomous.ToGoalTraj(drive, startPos);

        final TrajectorySequence toSpinner = BlueCarouselAutonomous.ToSpinner(drive, toGoal.end());

        final TrajectorySequence pickingUpDuck = PickingUpDuck(drive, toSpinner.end(), log);

        final TrajectorySequence cycleDuckGrab = CycleDuckGrab(drive, pickingUpDuck.end(), log, telemetry);

        final Trajectory toPark = BlueCarouselAutonomous.ToEnd(drive, cycleDuckGrab.end());

        final TrajectorySequence tryThis = TryThis(drive, new Pose2d(-53, 55, Math.toRadians(95)));

        telemetry.addData("Setup", "Finished");
        telemetry.update();

        final BarcodePositions pos = monitorMarkerWhileWaitForStart();


        //final BarcodePositions pos = monitorMarkerWhileWaitForStart();
        waitForStart();

        this.closeWebcam();
        drive.setPoseEstimate(pickingUpDuck.start());


        if (!isStopRequested()) {
            drive.followTrajectory(toGoal);

            drive.turnTo(dropOffPos.getHeading());

            this.dropOffItem(pos);

            drive.followTrajectorySequence(toSpinner);

            slide.setTargetLevel(HeightLevel.Down);

            spinner.spinOffBlueDuck();

            intake.setMotorPower(1);

            drive.followTrajectorySequence(pickingUpDuck);

            drive.followTrajectorySequence(cycleDuckGrab);

            intake.setMotorPower(0);

            drive.followTrajectory(toPark);


        }

    }

}
