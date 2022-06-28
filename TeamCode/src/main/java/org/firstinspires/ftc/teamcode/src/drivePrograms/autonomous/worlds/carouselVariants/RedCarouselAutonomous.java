package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.carouselVariants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.WorldsAutonomousProgram;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.LinearSlide;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.podservos.OdometryServosImpl;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "🟥Red Carousel Autonomous🟥", group = "RedCarousel")
public class RedCarouselAutonomous extends WorldsAutonomousProgram {
    static final Pose2d startPos = new Pose2d(-40, -65, Math.toRadians(0));
    static final Pose2d dropOffPos = new Pose2d(-30, -24, Math.toRadians(180));
    static final Pose2d parkPos = new Pose2d(-60, -35.5, Math.toRadians(90));
    static final Pose2d carouselSpinPos = new Pose2d(-65, -54, Math.toRadians(270));
    public static double LraiseAmmount = 0.04;
    public static double HraiseAmmount = 0.04;
    public static double RraiseAmmount = 0.04;
    public static double crossOffsetX = 8;
    public static double vMax = 15;
    public static double aMax = 10;
    public static double xAdjust = 0;
    public static double yAdjust = -0;
    public static double rotAdjust = -0;
    BarcodePositions detectedPos;
    public RedCarouselAutonomous() {
        super(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    public static TrajectorySequence ToSpinner(SampleMecanumDrive drive, Pose2d startPos, LinearSlide slide) {
        return drive.trajectorySequenceBuilder(startPos)
                .setConstraints((one, two, three, v) -> vMax, (one, two, three, a) -> aMax)

                .addSpatialMarker(parkPos.plus(startPos).vec().div(2), () -> slide.setTargetLevel(HeightLevel.Down))

                //Back away from goal
                // Cross Box
                .lineToConstantHeading(new Pose2d(parkPos.getX() + 5, parkPos.getY() + 10, Math.toRadians(265)).vec())

                // Cross Box to Carousel Spinner
                .splineToSplineHeading(carouselSpinPos.plus(new Pose2d(10, 10, Math.toRadians(-30))), Math.toRadians(-90))
                .splineToConstantHeading(carouselSpinPos.plus(new Pose2d(xAdjust, yAdjust, Math.toRadians(-rotAdjust))).vec(), Math.toRadians(-90))
                .build();
    }

    public static Trajectory ToEnd(SampleMecanumDrive drive, Pose2d startPos, LinearSlide slide) {
        return drive.trajectoryBuilder(startPos)
                //Park
                .addSpatialMarker(startPos.plus(parkPos).div(2).vec(), () -> slide.setTargetLevel(HeightLevel.Down))

                .lineToLinearHeading(parkPos)
                .build();
    }

    public static TrajectorySequence ToGoalTraj(SampleMecanumDrive drive, Pose2d startPos, LinearSlide slide, Executable<BarcodePositions> getPos) {
        return drive.trajectorySequenceBuilder(startPos)
                .setConstraints((one, two, three, v) -> vMax, (one, two, three, a) -> aMax)


                // Side in
                .lineToConstantHeading(parkPos.plus(new Pose2d(crossOffsetX, -10, 0)).vec())
                // Cross Box
                .splineToSplineHeading(new Pose2d(parkPos.getX() + crossOffsetX, parkPos.getY() + 10, dropOffPos.getHeading()), Math.toRadians(0))
                .addSpatialMarker(
                        new Pose2d(
                                parkPos.getX(), parkPos.getY(), dropOffPos.getHeading())
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
                .splineToSplineHeading(dropOffPos.plus(new Pose2d(6, -6, Math.toRadians(-6))), Math.toRadians(0))

                .setConstraints((one, two, three, v) -> 10, (one, two, three, a) -> aMax)
                .back(1)

                .build();
    }

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll(GenericOpModeTemplate.LeftWebcamName);

        final Executable<BarcodePositions> getPos = () -> detectedPos;

        final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        ((OdometryServosImpl) podServos).horizontalServo.setPosition(OdometryServosImpl.horizontalServoLowerPosition - HraiseAmmount);
        ((OdometryServosImpl) podServos).rightServo.setPosition(OdometryServosImpl.rightServoLowerPosition + RraiseAmmount);
        ((OdometryServosImpl) podServos).leftServo.setPosition(OdometryServosImpl.leftServoLowerPosition - LraiseAmmount);


        drive.setPoseEstimate(startPos);

        // From
        final TrajectorySequence toGoal = RedCarouselAutonomous.ToGoalTraj(drive, startPos, slide, getPos);

        final TrajectorySequence toSpinner = RedCarouselAutonomous.ToSpinner(drive, toGoal.end(), slide);

        final Trajectory toPark = RedCarouselAutonomous.ToEnd(drive, toSpinner.end(), slide);

        telemetry.addData("Setup", "Finished");
        telemetry.update();

        detectedPos = this.monitorMarkerWhileWaitForStart();

        if (!isStopRequested()) {

            drive.followTrajectorySequence(toGoal);

            this.dropOffItem(detectedPos);

            drive.followTrajectorySequence(toSpinner);

            slide.setTargetLevel(HeightLevel.Down);

            spinner.spinOffRedDuck();

            drive.followTrajectory(toPark);


        }
    }

}
