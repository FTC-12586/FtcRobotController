package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.podservos;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;

/**
 * A basic OpMode to test the pod servos
 */
@TeleOp(name = "Odometry Test")
@Disabled
public class OdometryTest extends GenericOpModeTemplate {


    @Override
    public void opModeMain() throws InterruptedException {
        this.initOdometryServos();
        OdometryServosImpl ps = (OdometryServosImpl) podServos;
        podServos.raise();
        waitForStart();

        if (isStopRequested()) {
            return;
        }

        telemetry.addData("Wiggling Left Servo", 'y');
        telemetry.update();
        ps.leftServo.setPosition(OdometryServosImpl.leftServoLowerPosition);
        Thread.sleep(500);
        ps.leftServo.setPosition(OdometryServosImpl.leftServoRaisePosition);
        Thread.sleep(500);

        telemetry.addData("Wiggling Right Servo", 'y');
        telemetry.update();
        ps.rightServo.setPosition(OdometryServosImpl.rightServoLowerPosition);
        Thread.sleep(500);
        ps.rightServo.setPosition(OdometryServosImpl.rightServoRaisePosition);
        Thread.sleep(500);

        telemetry.addData("Wiggling Horizontal Servo", 'y');
        telemetry.update();
        ps.horizontalServo.setPosition(OdometryServosImpl.horizontalServoLowerPosition);
        Thread.sleep(500);
        ps.horizontalServo.setPosition(OdometryServosImpl.horizontalServoRaisePosition);
        Thread.sleep(500);

        while (opModeIsActive() && !isStopRequested()) {
            podServos.raise();
            Thread.sleep(5000);
            podServos.lower();
            Thread.sleep(5000);
        }

    }
}
