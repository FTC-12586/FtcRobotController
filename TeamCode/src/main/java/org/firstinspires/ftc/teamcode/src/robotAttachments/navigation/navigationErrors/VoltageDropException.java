package org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationErrors;


import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.LocalizationAlgorithm;
import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtils;

public class VoltageDropException extends MovementException {

    private final double dropAmount;
    private Double initialVoltage = null;


    public VoltageDropException(double dropAmmnt) {
        super();
        this.dropAmount = Math.abs(dropAmmnt);
    }

    @Override
    public void call(double x, double y, double theta, double tolerance, Telemetry telemetry, LocalizationAlgorithm gps, Executable<Boolean> _isStopRequested, Executable<Boolean> _opModeIsActive, RobotVoltageSensor voltageSensor) throws MovementException {
        if (initialVoltage == null) {
            initialVoltage = voltageSensor.getVoltage();
        }

        if (voltageSensor.getVoltage() < (initialVoltage - dropAmount)) {
            final String args = "moveToPosition(" + x + ", " + y + ", " + theta + ", " + tolerance + ")\n";
            final String errorMsg = "In function call " + args + MiscUtils.getRelativeClassName(this) + " Error.\n";
            RobotLog.addGlobalWarningMessage(errorMsg);
            throw this;
        }
    }
}