package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.intake;

import org.firstinspires.ftc.teamcode.src.utills.Controllable;

public interface Intake extends Controllable<Void> {

    void setMotorPower(double power);

     void setFrontMotorPower(double power);

     void setBackMotorPower(double power);

}
