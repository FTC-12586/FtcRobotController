package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.carouselVariants.recovery;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.WorldsAutonomousProgram;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutonomousTemplate;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@SuppressWarnings("unused")
//@Config
@Autonomous(name = "☠🟥Red Carousel WH Park Autonomous🟥☠", group = "recovery")
public class RedCarouselAutonomousWHPark extends WorldsAutonomousProgram {
    static final Pose2d startPos = new Pose2d(-40, -65, Math.toRadians(0));
    static final Pose2d dropOffPos = new Pose2d(-33, -25, Math.toRadians(180));
    static final Pose2d parkPos = new Pose2d(-60, -35.5, Math.toRadians(270));
    static final Pose2d carouselSpinPos = new Pose2d(-65, -54, Math.toRadians(270));
    final static Pose2d warehouseCrossPos = new Pose2d(11, -46, 0);

    BarcodePositions detectedPos;

    public RedCarouselAutonomousWHPark() {
        super(GenericOpModeTemplate.LEDErrorColor);
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
        ((AutonomousTemplate) this).initAll();

        final Executable<BarcodePositions> getPos = () -> detectedPos;


        final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        drive.setPoseEstimate(startPos);

        // From
        final Trajectory toGoal = RedCarouselAutonomous.ToGoalTraj(drive, startPos, slide, getPos);

        final TrajectorySequence toSpinner = RedCarouselAutonomous.ToSpinner(drive, toGoal.end(), slide);

        final TrajectorySequence toPark = toEnd(drive, toSpinner.end());

        telemetry.addData("Setup", "Finished");
        telemetry.update();
        detectedPos = BarcodePositions.NotSeen;
        waitForStart();


        if (!isStopRequested()) {

            drive.followTrajectory(toGoal);
            drive.turnTo(dropOffPos.getHeading());

            this.dropOffItem(detectedPos);

            drive.followTrajectorySequence(toSpinner);
            slide.setTargetLevel(HeightLevel.Down);

            spinner.spinOffRedDuck();

            drive.followTrajectorySequence(toPark);

            this.driveOverBarriers();


        }
    }

}