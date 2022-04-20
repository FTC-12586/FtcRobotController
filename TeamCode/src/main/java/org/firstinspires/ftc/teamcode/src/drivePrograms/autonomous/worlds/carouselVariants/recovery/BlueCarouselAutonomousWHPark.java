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

//@Config
@Autonomous(name = "☠🟦Blue Carousel WH Park Autonomous🟦☠", group = "recovery")
public class BlueCarouselAutonomousWHPark extends WorldsAutonomousProgram {
    final static Pose2d startPos = new Pose2d(-34, 65, 0);
    final static Pose2d dropOffPos = new Pose2d(-27, 23.5, Math.toRadians(180));
    final static Pose2d warehouseCrossPos = new Pose2d(11, 46, 0);
    static BarcodePositions detectedPos = null;

    public BlueCarouselAutonomousWHPark() {
        super(GenericOpModeTemplate.LEDErrorColor);
    }

    public static TrajectorySequence toEnd(SampleMecanumDrive drive, Pose2d startPos) {
        return drive.trajectorySequenceBuilder(startPos)
                //Park
                .back(2)
                .lineToLinearHeading(warehouseCrossPos.plus(new Pose2d(5, 23, Math.toRadians(20))))
                .build();
    }


    @Override
    public void opModeMain() throws InterruptedException {

        final Executable<BarcodePositions> getPos = () -> BlueCarouselAutonomousWHPark.detectedPos;

        ((AutonomousTemplate) this).initAll();

        leds.setPattern(defaultColor);

        drive.setPoseEstimate(startPos);

        // From
        final Trajectory toGoal = BlueCarouselAutonomous.ToGoalTraj(drive, startPos, slide, getPos);

        final TrajectorySequence toSpinner = BlueCarouselAutonomous.ToSpinner(drive, toGoal.end(), slide);

        final TrajectorySequence toPark = toEnd(drive, toSpinner.end());

        telemetry.addData("Setup", "Finished");
        telemetry.update();

        detectedPos = BarcodePositions.NotSeen;

        waitForStart();

        if (!isStopRequested()) {

            drive.followTrajectory(toGoal);

            this.dropOffItem(detectedPos);

            drive.followTrajectorySequence(toSpinner);
            slide.setTargetLevel(HeightLevel.Down);

            spinner.spinOffBlueDuck();

            drive.followTrajectorySequence(toPark);
            this.driveOverBarriers();


        }
    }

}
