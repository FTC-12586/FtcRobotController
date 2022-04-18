package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.worlds;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.src.utills.enums.FreightFrenzyGameObject;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.TeleOpTemplate;

@TeleOp(name = "🟥Red Worlds Drive Program🟥")
public class RedWorldsDriveProgram extends TeleOpTemplate {
    private final ElapsedTime SpaceBarColorTimer = new ElapsedTime();
    protected BlinkinPattern defaultColor;
    protected BlinkinPattern currentPattern;
    private boolean x_depressed = true;
    private boolean tapeMeasureCtrl = false;

    private boolean spaceBarDepressed = true;

    public RedWorldsDriveProgram() {
        defaultColor = BlinkinPattern.RED;
        currentPattern = this.defaultColor;
    }


    public void opModeMain() throws InterruptedException {
        this.initAll();

        leds.setPattern(currentPattern);

        telemetry.addData("Initialization", "finished");
        telemetry.update();

        System.gc();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            //Declan's controls
            {
                if (tapeMeasureCtrl) {
                    turret.gamepadControl(gamepad2, gamepad1);

                } else {
                    driveTrain.gamepadControl(gamepad1, gamepad2);
                }

                if (!gamepad1.x) {
                    x_depressed = true;
                }

                if (gamepad1.x && x_depressed) {
                    x_depressed = false;
                    tapeMeasureCtrl = !tapeMeasureCtrl;
                    driveTrain.halt();
                    turret.halt();
                }


            }


            //Eli's controls
            {
                spinner.gamepadControl(gamepad1, gamepad2);


                //Handles Linear Slide Control
                slide.gamepadControl(gamepad1, gamepad2);

                //Intake Controls
                // The intake may propose a pattern
                BlinkinPattern proposedPattern = FreightFrenzyGameObject.getLEDColorFromItem(outtake.gamepadControl(gamepad1, gamepad2));

                // Check SPace Bar, if pressed, reset space bar timer
                if (spaceBar.isPressed() && spaceBarDepressed) {
                    SpaceBarColorTimer.reset();
                    spaceBarDepressed = false;
                }
                if (!spaceBar.isPressed()) {
                    spaceBarDepressed = true;
                }

                intake.gamepadControl(gamepad1, gamepad2);

                //If space bar timer is less than 1, turn LEDS off
                //This overrides the intakes' proposed pattern
                if (SpaceBarColorTimer.seconds() < 1 && SpaceBarColorTimer.seconds() > .25) {
                    intake.setFrontMotorPower(0);
                    proposedPattern = BlinkinPattern.BLACK;
                }

                if (proposedPattern != null && proposedPattern != currentPattern) {
                    currentPattern = proposedPattern;
                    leds.setPattern(currentPattern);
                } else {
                    if (currentPattern != defaultColor) {
                        currentPattern = defaultColor;
                        leds.setPattern(this.defaultColor);
                    }
                }

            }

            Thread.yield();

        }
    }
}


