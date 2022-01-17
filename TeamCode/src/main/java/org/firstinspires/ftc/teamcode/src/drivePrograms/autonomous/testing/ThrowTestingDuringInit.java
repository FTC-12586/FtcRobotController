package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.utills.TeleOpTemplate;

/**
 * A Autonomous to test that utility in {@link org.firstinspires.ftc.teamcode.src.utills.GenericOpModeTemplate} for extra info from stack traces works properly
 */
@Disabled
@Autonomous(name = "throwTest")
public class ThrowTestingDuringInit extends TeleOpTemplate {
    public void opModeMain() throws InterruptedException {
        this.initAll();
    }
}