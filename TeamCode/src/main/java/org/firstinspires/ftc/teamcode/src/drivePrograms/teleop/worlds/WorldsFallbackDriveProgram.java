package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.worlds;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.TeleOpTemplate;

@TeleOp(name = "☠Recovery Drive Program☠")
public class WorldsFallbackDriveProgram extends TeleOpTemplate {
    private boolean tapeMeasureCtrl = false;
    private boolean x_depressed = true;

    @Override
    public void opModeMain() throws InterruptedException {

        //Redundant Initialization
        {

            try {
                this.initDriveTrain();
            } catch (RuntimeException e) {
                RobotLog.addGlobalWarningMessage("Drivetrain failed to init");
                RobotLog.ee("Drivetrain init failed", e, "Continuing OpMode");
            }

            try {
                this.initLinearSlide();
            } catch (RuntimeException e) {
                RobotLog.addGlobalWarningMessage("Linear Slide failed to init");
                RobotLog.ee("Linear Slide init failed", e, "Continuing OpMode");
            }

            try {
                this.initIntake();
            } catch (RuntimeException e) {
                RobotLog.addGlobalWarningMessage("Intake failed to init");
                RobotLog.ee("Intake init failed", e, "Continuing OpMode");
            }

            try {
                this.initOuttake();
            } catch (RuntimeException e) {
                RobotLog.addGlobalWarningMessage("Outtake failed to init");
                RobotLog.ee("Outtake init failed", e, "Continuing OpMode");
            }

            try {
                this.initOdometryServos();
                podServos.raise();
            } catch (RuntimeException e) {
                RobotLog.addGlobalWarningMessage("Pod Servos failed to init");
                RobotLog.ee("Odometry Servos init failed", e, "Continuing OpMode");
            }

            try {
                this.initTapeMeasureTurret();
            } catch (RuntimeException e) {
                RobotLog.addGlobalWarningMessage("Tape Measure Turret failed to init");
                RobotLog.ee("Tape Measure Turret init failed", e, "Continuing OpMode");
            }

            try {
                this.initSpinner();
            } catch (RuntimeException e) {
                RobotLog.addGlobalWarningMessage("Spinner failed to init");
                RobotLog.ee("Spinner init failed", e, "Continuing OpMode");
            }

            try {
                this.initLEDS();
                leds.setPattern(GenericOpModeTemplate.LEDErrorColor);
            } catch (RuntimeException e) {
                RobotLog.addGlobalWarningMessage("LEDS failed to init");
                RobotLog.ee("LEDS failed", e, "Continuing OpMode");
            }

        }

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            //Declan's controls
            {
                if (tapeMeasureCtrl) {
                    try {
                        turret.gamepadControl(gamepad2, gamepad1);
                    } catch (RuntimeException e) {
                        RobotLog.e("Turret failed to be controlled");
                        try {
                            turret.halt();
                        } catch (RuntimeException ignored) {}
                    }
                } else {
                    try {
                        driveTrain.gamepadControl(gamepad1, gamepad2);
                    } catch (RuntimeException e) {
                        RobotLog.e("Drive Train failed to be controlled");
                        try {
                            driveTrain.halt();
                        } catch (RuntimeException ignored) {}
                    }
                }

                if (!gamepad1.x) {
                    x_depressed = true;
                }

                if (gamepad1.x && x_depressed) {
                    tapeMeasureCtrl = !tapeMeasureCtrl;

                    try {
                        driveTrain.halt();
                    } catch (RuntimeException e) {
                        RobotLog.e("Drive Train failed to be controlled");
                        try {
                            driveTrain.halt();
                        } catch (RuntimeException ignored) {}
                    }

                    try {
                        turret.halt();
                    } catch (RuntimeException e) {
                        RobotLog.e("Turret failed to be controlled");
                        try {
                            turret.halt();
                        } catch (RuntimeException ignored) {
                        }
                    }
                }

            }

            //Eli's controls
            {
                try {
                    spinner.gamepadControl(gamepad1, gamepad2);
                } catch (RuntimeException e) {
                    RobotLog.e("Spinner failed to be controlled");
                    try {
                        spinner.halt();
                    } catch (RuntimeException ignored) {}
                }

                try {
                    slide.gamepadControl(gamepad1, gamepad2);
                } catch (RuntimeException e) {
                    RobotLog.e("Slide failed to be controlled");
                    try {
                        slide.halt();
                    } catch (RuntimeException ignored) {
                    }
                }

                try {
                    outtake.gamepadControl(gamepad1, gamepad2);
                } catch (RuntimeException e) {
                    RobotLog.e("Outtake failed to be controlled");
                    try {
                        outtake.halt();
                    } catch (RuntimeException ignored) {}
                }

                try {
                    intake.gamepadControl(gamepad1, gamepad2);
                } catch (RuntimeException e) {
                    RobotLog.e("Intake failed to be controlled");
                    try {
                        intake.halt();
                    } catch (RuntimeException ignored) {}
                }
            }
            Thread.yield();
        }
    }
}
