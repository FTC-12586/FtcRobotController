package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutonomousTemplate;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "🟦Blue Carousel Autonomous🟦")
public class BlueCarouselAutonomous extends AutonomousTemplate {
    final static Pose2d startPos = new Pose2d(-34, 65, 0);
    final static Pose2d dropOffPos = new Pose2d(-27, 23.5, Math.toRadians(180));
    final static Pose2d carouselSpinPos = new Pose2d(-61, 51, Math.toRadians(90));
    final static Pose2d parkPos = new Pose2d(-60, 35.5, Math.toRadians(90));

    @Override
    public void opModeMain() throws InterruptedException {
        this.initLinearSlide();
        this.initOdometryServos();
        this.initLEDS();
        this.initSpinner();
        podServos.lower();

        final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        drive.setPoseEstimate(startPos);


        waitForStart();

        // From
        final TrajectorySequence toGoal = drive.trajectorySequenceBuilder(startPos)
                // Side in
                .lineToConstantHeading(new Pose2d(parkPos.getX() + 12, parkPos.getY() + 15, 0).vec())
                // Cross Box
                .splineToConstantHeading(parkPos.plus(new Pose2d(12, -15, 0)).vec(), 0)
                //Approach Goal
                .splineToSplineHeading(dropOffPos.plus(new Pose2d(0, 0, Math.toRadians(-0))), 0)
                .build();

        final TrajectorySequence toSpinner = drive.trajectorySequenceBuilder(toGoal.end())
                // Cross Box
                .lineToLinearHeading(new Pose2d(parkPos.getX() + 5, parkPos.getY() - 15, Math.toRadians(270)))

                //.setConstraints((v, pose2d, pose2d1, pose2d2) -> 10, (v, pose2d, pose2d1, pose2d2) -> 20)
                // To Carousel Spinner
                .lineTo(carouselSpinPos.vec().plus(new Vector2d(5)))
                .build();

        final TrajectorySequence toPark = drive.trajectorySequenceBuilder(toSpinner.end())
                //Park
                .lineTo(parkPos.vec().plus(new Vector2d(5, +2)))
                .build();

        if (!isStopRequested()) {

            drive.followTrajectorySequence(toGoal);
            drive.turnTo(dropOffPos.getHeading());
            slide.setTargetLevel(HeightLevel.TopLevel);


            Thread.sleep(1000);

            slide.setTargetLevel(HeightLevel.BottomLevel);

            Thread.sleep(1000);

            drive.followTrajectorySequence(toSpinner);

            spinner.spinOffRedDuck();

            drive.followTrajectorySequence(toPark);


        }
    }

}