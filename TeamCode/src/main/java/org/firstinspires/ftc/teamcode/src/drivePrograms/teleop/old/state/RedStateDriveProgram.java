package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.old.state;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.TripWireDistanceSensor;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.TeleOpTemplate;

@TeleOp(name = "\uD83D\uDFE5 Red State Drive Program \uD83D\uDFE5")
public class RedStateDriveProgram extends TeleOpTemplate {
    final BlinkinPattern defaultColor = BlinkinPattern.RED;
    private final ElapsedTime yTimer = new ElapsedTime();

    private final ElapsedTime slideResetTimer = new ElapsedTime();
    boolean manualSlideControl = false;

    boolean y_depressed2 = true;
    boolean dPadUpDepressed = true;
    boolean dPadDownDepressed = true;
    HeightLevel currentLevel = HeightLevel.Down;
    private boolean resetSlide = false;

    BlinkinPattern currentColor = defaultColor;
    TripWireDistanceSensor distanceSensor;

    @Nullable
    private Void callBack() {
        this.leds.setPattern(BlinkinPattern.BLACK);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException ignored) {
        }
        this.leds.setPattern(currentColor);
        return null;

    }

    public void opModeMain() throws InterruptedException {
        this.initAll();
        distanceSensor = new TripWireDistanceSensor(hardwareMap, "distance_sensor", 8, this::callBack, this::opModeIsActive, this::isStopRequested);
        distanceSensor.start();
        leds.setPattern(defaultColor);

        slide.autoMode();

        telemetry.addData("Initialization", "finished");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            //Declan's controls
            {
                driveTrain.gamepadControl(gamepad1, null);

                //Declan Speed Modifiers
                if (gamepad1.b) {
                    driveTrain.setDrivePowerMult(0.6);
                }
                if (gamepad1.y) {
                    driveTrain.setDrivePowerMult(1);

                }
                if (gamepad1.a) {
                    driveTrain.setDrivePowerMult(0.3);
                }

            }

            //Eli's controls
            {
                //Handles Linear Slide Control
                {
                    if (!resetSlide) {
                        if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                            manualSlideControl = true;
                            int pos = (int) (Math.abs(slide.getEncoderCount()) + (100 * -gamepad2.left_stick_y));
                            telemetry.addData("Pos", pos);
                            telemetry.update();
                            if (pos < -100) pos = -100;
                            if (pos > HeightLevel.getEncoderCountFromEnum(HeightLevel.TopLevel))
                                pos = HeightLevel.getEncoderCountFromEnum(HeightLevel.TopLevel);
                            slide.setTargetPosition(pos);
                        }

                        if (gamepad2.dpad_left) {
                            slide.setTargetLevel(HeightLevel.CappingUp);
                        }

                        if (gamepad2.dpad_right) {
                            slide.setTargetLevel(HeightLevel.CappingDown);
                        }

                        if (!gamepad2.dpad_up) {
                            dPadUpDepressed = true;
                        }

                        if (gamepad2.dpad_up && dPadUpDepressed) {
                            if (manualSlideControl) {
                                manualSlideControl = false;
                                currentLevel = HeightLevel.getClosestLevel(slide.getEncoderCount());
                            }
                            dPadUpDepressed = false;
                            currentLevel = currentLevel.add(1);
                            slide.setTargetLevel(currentLevel);

                        }

                        if (!gamepad2.dpad_down) {
                            dPadDownDepressed = true;
                        }

                        if (gamepad2.dpad_down && dPadDownDepressed) {
                            if (manualSlideControl) {
                                manualSlideControl = false;
                                currentLevel = HeightLevel.getClosestLevel(slide.getEncoderCount());
                            }
                            dPadDownDepressed = false;
                            currentLevel = currentLevel.subtract(1);
                            slide.setTargetLevel(currentLevel);
                        }

                        if (gamepad2.right_bumper && gamepad2.left_bumper) {
                            slide.teleopMode();
                            slideResetTimer.reset();
                            resetSlide = true;
                        }
                    } else if (slideResetTimer.seconds() > 0.5 && slideResetTimer.seconds() < 0.6) {
                        slide.setMotorPower(0.3);
                    } else if (slideResetTimer.seconds() > 1 && resetSlide) {
                        resetSlide = false;
                        slide.autoMode();
                        slide.resetEncoder();
                    }

                }

                //Intake Controls
                {
                    if (Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) > 0.01) {
                        intake.setMotorPower(gamepad2.left_trigger - gamepad2.right_trigger);
                        BlinkinPattern o = intake.getLEDPatternFromFreight();
                        if (o == null || !intake.isClosed()) {
                            if (currentColor != defaultColor) {
                                leds.setPattern(defaultColor);
                                currentColor = defaultColor;
                            }
                        } else {
                            if (currentColor != o) {
                                leds.setPattern(o);
                                currentColor = o;
                            }
                        }
                    } else {
                        intake.setMotorPower(0);
                    }
                }

                //Out take controls
                {

                    //90 deg
                    if (gamepad2.y) {
                        y_depressed2 = false;
                        intake.setServoOpen();
                        leds.setPattern(defaultColor);
                        currentColor = defaultColor;
                        yTimer.reset();
                    }

                    //down
                    if (gamepad2.a) {
                        intake.setServoPos(1);
                        yTimer.reset();
                    }

                    //135 deg
                    if (gamepad2.b) {
                        intake.setServoPos(.88);
                        yTimer.reset();
                    }

                    if (yTimer.seconds() > 1.25) {
                        intake.setServoClosed();
                    }
                }

                //Carousel Spinner
                {
                    if (Math.abs(gamepad2.right_stick_x) > 0.1) {
                        if (gamepad2.right_stick_x > 0) {
                            spinner.setPowerBlueDirection();
                        } else {
                            spinner.setPowerRedDirection();
                        }
                    } else {
                        spinner.stop();
                    }
                }

            }


        }
    }
}

