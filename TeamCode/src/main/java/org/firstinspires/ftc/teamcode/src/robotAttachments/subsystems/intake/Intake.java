package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.intake;

import org.firstinspires.ftc.teamcode.src.utills.Controllable;

/**
 * An interface for intakes with two intake motors
 */
public interface Intake extends Controllable<Void> {

    /**
     * Sets power to both of the underlying {@link com.qualcomm.robotcore.hardware.DcMotor} object
     *
     * @param power The power to run at in range [-1,1]
     */
    void setMotorPower(double power);

    /**
     * Sets power to both the front {@link com.qualcomm.robotcore.hardware.DcMotor} object
     *
     * @param power The power to run at in range [-1,1]
     */
    void setFrontMotorPower(double power);

    /**
     * Sets power to both the back {@link com.qualcomm.robotcore.hardware.DcMotor} object
     *
     * @param power The power to run at in range [-1,1]
     */
    void setBackMotorPower(double power);

}
