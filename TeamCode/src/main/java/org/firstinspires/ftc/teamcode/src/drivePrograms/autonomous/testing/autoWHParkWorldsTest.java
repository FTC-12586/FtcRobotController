package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.WorldsAutonomousProgram;

@Disabled
@Autonomous(name = "Auto WH Park Test")
public class autoWHParkWorldsTest extends WorldsAutonomousProgram {

    public autoWHParkWorldsTest() {
        super(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        closeWebcam();
        driveOverBarriersDistanceSensor();
        drive.halt();
    }
}
