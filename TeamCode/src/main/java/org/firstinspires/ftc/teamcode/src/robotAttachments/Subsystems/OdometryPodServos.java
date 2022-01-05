package org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OdometryPodServos {

    private static final double rightServoRaisePosition = 0.43;
    private static final double rightServoLowerPosition = .72;

    private static final double leftServoRaisePosition = 0.8;
    private static final double leftServoLowerPosition = 0.54;

    private static final double horizontalServoRaisePosition = 0.4;
    private static final double horizontalServoLowerPosition = 0.69;

    final Servo horizontalServo;
    final Servo leftServo;
    final Servo rightServo;

    public OdometryPodServos(HardwareMap hardwareMap, String rightServoName, String leftServoName, String horizontalServoName) {
        this.horizontalServo = hardwareMap.servo.get(horizontalServoName);
        this.rightServo = hardwareMap.servo.get(rightServoName);
        this.leftServo = hardwareMap.servo.get(leftServoName);
    }

    public void lower() {
        horizontalServo.setPosition(horizontalServoLowerPosition);
        rightServo.setPosition(rightServoLowerPosition);
        leftServo.setPosition(leftServoLowerPosition);
    }

    public void raise(){
        horizontalServo.setPosition(horizontalServoRaisePosition);
        rightServo.setPosition(rightServoRaisePosition);
        leftServo.setPosition(leftServoRaisePosition);
    }
}
