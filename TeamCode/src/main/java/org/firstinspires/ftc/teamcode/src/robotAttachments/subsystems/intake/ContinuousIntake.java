package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.intake;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Continuous intake with two intake motors
 */
public class ContinuousIntake implements Intake {
    /**
     * The power for going forward for the front motor
     */
    private final static double frontMotorForwardPower = -1;

    /**
     * The power for going forward for the back motor
     */
    private final static double backMotorForwardPower = 1;

    /**
     * Front DcMotor Object
     */
    private final DcMotor frontIntakeMotor;

    /**
     * Back DcMotor Object
     */
    private final DcMotor backIntakeMotor;


    /**
     * Initializes Class
     *
     * @param hardwareMap    A opMode hardware map
     * @param frontMotorName Name of the front intake motor
     * @param backMotorName  Name of the back intake motor
     */
    public ContinuousIntake(HardwareMap hardwareMap, String frontMotorName, String backMotorName) {
        frontIntakeMotor = hardwareMap.dcMotor.get(frontMotorName);
        frontIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backIntakeMotor = hardwareMap.dcMotor.get(backMotorName);
        backIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    /**
     * @param power a variable input for the power of the intake motor
     *              this sets the power of the intake motor to the power variable input
     */
    public void setMotorPower(double power) {
        frontIntakeMotor.setPower(power * frontMotorForwardPower);
        backIntakeMotor.setPower(power * backMotorForwardPower);
    }

    /**
     * {@inheritDoc}
     */
    public void halt() {
        setMotorPower(0);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Void gamepadControl(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) {

        if (Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) > 0.01) {

            this.setMotorPower(gamepad2.left_trigger - gamepad2.right_trigger);
        } else {
            this.setMotorPower(0);
        }
        return null;
    }

    /**
     * {@inheritDoc}
     */
    public void setFrontMotorPower(double power) {
        frontIntakeMotor.setPower(power);
    }

    /**
     * {@inheritDoc}
     */
    public void setBackMotorPower(double power) {
        backIntakeMotor.setPower(power);
    }

}
