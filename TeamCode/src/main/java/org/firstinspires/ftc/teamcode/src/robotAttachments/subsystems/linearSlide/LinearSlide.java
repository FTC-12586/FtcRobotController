package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide;

import org.firstinspires.ftc.teamcode.src.utills.Controllable;

public interface LinearSlide extends Controllable<Void> {
    void setTargetPosition(int targetPosition);

    void setTargetLevel(HeightLevel level);

    void waitOn() throws InterruptedException;

    boolean isAtPosition();

    boolean isAtPosition(int tolerance);

    int getEncoderCount();

    void resetEncoder();

    void teleopMode();

    void autoMode();

    void reverseMotor();

    void setMotorPower(double power);

    int getTargetHeight();

}
