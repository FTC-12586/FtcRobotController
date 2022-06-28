package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.tapemeasureturret;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.TeleOpTemplate;

/**
 * A basic OpMode to test the Tape Measure Turret
 */
@Disabled
@TeleOp(name = "\u0000TapeMeasureTurretTest")
public class TapeMeasureTurretTest extends TeleOpTemplate {

    @Override
    public void opModeMain() {
        final TapeMeasureTurretImpl turret = new TapeMeasureTurretImpl(hardwareMap, "tape_measure", "pitch", "yaw");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            turret.gamepadControl(gamepad1, gamepad2);
            Thread.yield();
        }

    }
}
