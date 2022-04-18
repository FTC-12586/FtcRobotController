package org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.odometry.examples;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;

import java.io.File;

/**
 * Odometry system calibration. Run this OpMode to generate the necessary constants to calculate the robot's global position on the field.
 * The Global Positioning Algorithm will not function and will throw an error if this program is not run first
 *
 * @author Sarthak
 * @since 6/1/2019
 */
@Disabled
@TeleOp(name = "Odometry System Calibration", group = "Calibration")
public class OdometryCalibration extends GenericOpModeTemplate {

    private final static double PIVOT_SPEED = 0.05;
    //The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
    private final static double COUNTS_PER_INCH = 1892.3724283364;
    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    private final static String rfName = GenericOpModeTemplate.frontRightName;
    private final static String rbName = GenericOpModeTemplate.backRightName;
    private final static String lfName = GenericOpModeTemplate.frontLeftName;
    private final static String lbName = GenericOpModeTemplate.backLeftName;
    private final static String verticalLeftEncoderName = GenericOpModeTemplate.backIntakeMotorName;
    private final static String verticalRightEncoderName = GenericOpModeTemplate.frontIntakeMotorName;
    private final static String horizontalEncoderName = GenericOpModeTemplate.horizontalOdometryName;
    //Text files to write the values to. The files are stored in the robot controller
    // under Internal Storage\FIRST\settings
    private final static File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private final static File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");
    private final ElapsedTime timer = new ElapsedTime();
    //Drive motors
    private DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    private DcMotor verticalLeft, verticalRight, horizontal;
    //IMU Sensor
    private BNO055IMU imu;

    @Override
    public void opModeMain() {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);
        this.initOdometryServos();
        podServos.lower();
        //Initialize IMU hardware map value. PLEASE UPDATE THIS VALUE TO MATCH YOUR CONFIGURATION
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
        telemetry.update();

        //Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();

        waitForStart();


        //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        double frPower = PIVOT_SPEED;
        double flPower = PIVOT_SPEED;
        double brPower = PIVOT_SPEED;
        double blPower = PIVOT_SPEED;

        //MUST TURN 90 DEGREES EXACTLY
        //PLAY WITH SPEED AND DEGREE OF TURN TO GET THE ROBOT TO END UP AS CLOSE TO 90 DEGREES AS POSSIBLE
        while (getZAngle() < 89.5 && opModeIsActive()) {
            right_front.setPower(frPower);
            right_back.setPower(brPower);
            left_front.setPower(flPower);
            left_back.setPower(blPower);
            if (getZAngle() < 60) {
                right_front.setPower(frPower);
                right_back.setPower(brPower);
                left_front.setPower(flPower);
                left_back.setPower(blPower);
            } else {
                right_front.setPower(frPower / 2.0);
                right_back.setPower(brPower / 2.0);
                left_front.setPower(flPower / 2.0);
                left_back.setPower(blPower / 2.0);
            }

            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
        }

        //Stop the robot
        setPowerAll(0, 0, 0, 0);
        timer.reset();
        while (timer.milliseconds() < 1000 && opModeIsActive()) {
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = getZAngle();



        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
       */

        double encoderDifference = Math.abs(-verticalLeft.getCurrentPosition()) + (Math.abs(verticalRight.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference / angle;

        double wheelBaseSeparation = (2 * 90 * verticalEncoderTickOffsetPerDegree) / (Math.PI * COUNTS_PER_INCH);

        double horizontalTickOffset = horizontal.getCurrentPosition() / Math.toRadians(getZAngle());

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));


        while (opModeIsActive()) {

            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.addData("Vertical Left Position", -verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical Right Position", verticalRight.getCurrentPosition());
            telemetry.addData("Horizontal Position", horizontal.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);


            telemetry.addData("Vertical Left Position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical Right Position", verticalRight.getCurrentPosition());
            telemetry.addData("Horizontal Position", horizontal.getCurrentPosition());

            //Update values
            telemetry.update();
        }
    }


    private void initHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName) {
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*
        Reverse the direction of the odometry wheels. THIS WILL CHANGE FOR EACH ROBOT. Adjust the direction (as needed) of each encoder wheel
        such that when the verticalLeft and verticalRight encoders spin forward, they return positive values, and when the
        horizontal encoder travels to the right, it returns positive value
        */

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();

    }

    /**
     * Gets the orientation of the robot using the REV IMU
     *
     * @return the angle of the robot
     */
    private double getZAngle() {
        return (-imu.getAngularOrientation().firstAngle);
    }

    /**
     * Sets power to all four drive motors
     *
     * @param rf power for right front motor
     * @param rb power for right back motor
     * @param lf power for left front motor
     * @param lb power for left back motor
     */
    private void setPowerAll(double rf, double rb, double lf, double lb) {
        right_front.setPower(rf);
        right_back.setPower(rb);
        left_front.setPower(lf);
        left_back.setPower(lb);
    }

}
