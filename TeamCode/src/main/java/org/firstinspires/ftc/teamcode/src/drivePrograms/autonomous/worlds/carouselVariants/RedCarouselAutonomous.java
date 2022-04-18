package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.carouselVariants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.WorldsAutonomousProgram;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "🟥Red Carousel Autonomous🟥", group = "RedCarousel")
public class RedCarouselAutonomous extends WorldsAutonomousProgram {
    static final Pose2d startPos = new Pose2d(-40, -65, Math.toRadians(0));
    static final Pose2d dropOffPos = new Pose2d(-33, -25, Math.toRadians(180));
    static final Pose2d parkPos = new Pose2d(-60, -35.5, Math.toRadians(270));
    static final Pose2d carouselSpinPos = new Pose2d(-65, -54, Math.toRadians(270));

    public RedCarouselAutonomous() {
        super(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    public static Trajectory ToGoalTraj(SampleMecanumDrive drive, Pose2d startPos) {
        return drive.trajectoryBuilder(startPos)
                // Side in
                .lineToConstantHeading(parkPos.plus(new Pose2d(12, -15)).vec())
                // Cross Box
                .splineToConstantHeading(parkPos.plus(new Pose2d(12, 19, 0)).vec(), 0)
                //Approach Goal
                .splineToSplineHeading(dropOffPos.plus(new Pose2d(6, 0, Math.toRadians(-16))), 0)
                //Final End Point
                .splineToConstantHeading(dropOffPos.plus(new Pose2d(8, -5, Math.toRadians(-16))).vec(), 0)
                .build();
    }

    public static TrajectorySequence ToSpinner(SampleMecanumDrive drive, Pose2d startPos) {
        return drive.trajectorySequenceBuilder(startPos)
                // Cross Box
                .lineToLinearHeading(new Pose2d(parkPos.getX() + 5, parkPos.getY() + 15, Math.toRadians(270)))
                //.setConstraints((v, pose2d, pose2d1, pose2d2) -> 10, (v, pose2d, pose2d1, pose2d2) -> 20)
                // To Carousel Spinner
                .lineTo(carouselSpinPos.vec().plus(new Vector2d(5)))
                .build();
    }

    public static Trajectory ToEnd(SampleMecanumDrive drive, Pose2d startPos) {
        return drive.trajectoryBuilder(startPos)
                //Park
                .lineTo(parkPos.vec().plus(new Vector2d(5)))
                .build();
    }

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll(GenericOpModeTemplate.LeftWebcamName);
        RobotLog.v("Completed initialization");

        final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        drive.setPoseEstimate(startPos);

        // From
        final Trajectory toGoal = RedCarouselAutonomous.ToGoalTraj(drive, startPos);

        final TrajectorySequence toSpinner = RedCarouselAutonomous.ToSpinner(drive, toGoal.end());

        final Trajectory toPark = RedCarouselAutonomous.ToEnd(drive, toSpinner.end());

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

            drive.followTrajectory(toPark);


        }
    }

}
