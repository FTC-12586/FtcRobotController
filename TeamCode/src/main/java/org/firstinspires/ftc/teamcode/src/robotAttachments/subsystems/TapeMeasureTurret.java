package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.src.utills.Controllable;

public class TapeMeasureTurret implements Controllable {
    private final CRServo tapeMeasure;
    private final CRServo pitch;
    private final CRServo yaw;

    public TapeMeasureTurret(HardwareMap hardwareMap, String tapeMeasureServoName, String pitchServoName, String yawServoName) {
        this.tapeMeasure = hardwareMap.crservo.get(tapeMeasureServoName);
        this.pitch = hardwareMap.crservo.get(pitchServoName);
        this.yaw = hardwareMap.crservo.get(yawServoName);
    }

    @Override
    public Object gamepadControl(Gamepad gamepad1, Gamepad gamepad2) {
        double tp = gamepad2.right_stick_y;
        double pp = gamepad2.right_stick_y;
        double yp = gamepad2.right_stick_y;

        tapeMeasure.setPower(-tp);
        pitch.setPower(pp);
        yaw.setPower(yp);

        return null;
    }
}