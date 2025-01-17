package org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.odometry.examples;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.odometry.ThreeWheelOdometry;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtils;


/**
 * Example OpMode that runs the GlobalCoordinatePosition thread and accesses the (x, y, theta) coordinate values
 *
 * @author Sarthak
 * @since 6/1/2019
 */
@Disabled
@TeleOp(name = "Global Coordinate Position Test", group = "Calibration")
public class GlobalCoordinatePositionUpdateSample extends LinearOpMode {

    //The amount of encoder ticks for each inch the robot moves. This will change for each robot and needs to be changed here
    private final static double COUNTS_PER_INCH = 1892.3724283364;
    //Hardware Map Names for drive motors and odometry wheels.
    private final static String rfName = "back_left";
    private final static String rbName = "front_left";
    private final static String lfName = "back_right";
    private final static String lbName = "front_right";
    private final static String verticalLeftEncoderName = rbName;
    private final static String verticalRightEncoderName = lbName;
    private final static String horizontalEncoderName = lfName;
    //Odometry encoder wheels
    private DcMotor verticalRight, verticalLeft, horizontal;
    private BNO055IMU imu;
    private DcMotor right_front;
    private DcMotor right_back;
    private DcMotor left_front;
    private DcMotor left_back;

    @Override
    public void runOpMode() throws InterruptedException {

        //Assign the hardware map to the odometry wheels
        verticalLeft = hardwareMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);

        //Reset the encoders
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
        Reverse the direction of the odometry wheels. THIS WILL CHANGE FOR EACH ROBOT. Adjust the direction (as needed) of each encoder wheel
        such that when the verticalLeft and verticalRight encoders spin forward, they return positive values, and when the
        horizontal encoder travels to the right, it returns positive value
        */
        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set the mode of the odometry encoders to RUN_WITHOUT_ENCODER
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        telemetry.clear();

        initHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
        ThreeWheelOdometry globalPositionUpdate = new ThreeWheelOdometry(verticalLeft, verticalRight, horizontal, 75, this::opModeIsActive, this::isStopRequested);
        globalPositionUpdate.start();


        //Init complete
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        double PIVOT_SPEED = 0.2;
        try {
            while (opModeIsActive() && !isStopRequested()) {
                double frPower = PIVOT_SPEED;
                double flPower = PIVOT_SPEED;
                double brPower = PIVOT_SPEED;
                double blPower = PIVOT_SPEED;
                if (getZAngle() < 90 && opModeIsActive()) {
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
                } else {
                    PIVOT_SPEED = 0;
                    frPower = PIVOT_SPEED;
                    flPower = PIVOT_SPEED;
                    brPower = PIVOT_SPEED;
                    blPower = PIVOT_SPEED;
                    right_front.setPower(frPower);
                    right_back.setPower(brPower);
                    left_front.setPower(flPower);
                    left_back.setPower(blPower);
                }
                //Display Global (x, y, theta) coordinates
                telemetry.addData("X Position", (Math.round((globalPositionUpdate.getX()) * 100.0)) / 100.0);
                telemetry.addData("Y Position", Math.round((globalPositionUpdate.getY()) * 100.0) / 100.0);
                telemetry.addData("Orientation (Degrees, ODO)", Math.round(globalPositionUpdate.getRot() * 100.0) / 100.0);
                telemetry.addData("Orientation (Degrees, IMU)", getZAngle());
                telemetry.addData("Thread Active", globalPositionUpdate.isAlive());
                telemetry.update();

            }
        } catch (Exception e) {
            telemetry.addData("Error", MiscUtils.getStackTraceAsString(e));
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) Thread.sleep(20);
        }

        //Stop the thread
        globalPositionUpdate.end();
    }

    private double getZAngle() {
        return (-imu.getAngularOrientation().firstAngle);
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

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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


}