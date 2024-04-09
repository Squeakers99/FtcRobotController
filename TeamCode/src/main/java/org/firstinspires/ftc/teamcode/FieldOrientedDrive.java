package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
public class FieldOrientedDrive extends LinearOpMode {

    //Declare Drivetrain Variables
    public DcMotor motorFL;
    public DcMotor motorFR;
    public DcMotor motorBL;
    public DcMotor motorBR;

    //Declare the Orientation variables
    BNO055IMU imu;
    Orientation angles = new Orientation();

    //Declares the variables for the yaw of the robot
    double initYaw;
    double adjustedYaw;

    @Override
    public void runOpMode() {
        //Configuration and initialization of the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        initYaw = angles.firstAngle;

        // Define and Initialize Motors
        motorFL  = hardwareMap.get(DcMotor.class, "front_left");
        motorFR  = hardwareMap.get(DcMotor.class, "front_right");
        motorBL  = hardwareMap.get(DcMotor.class, "back_left");
        motorBR  = hardwareMap.get(DcMotor.class, "back_right");

        //Sets all motors to brake mode
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reverse the left side motors
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Calculates the angle of the robot
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            
            //Calculates the yaw of the robot (attempted)
            adjustedYaw = angles.firstAngle - initYaw;
            double zerodYaw = -initYaw+angles.firstAngle;

            //Gamepad inputs
            double movementX = gamepad1.left_stick_x;
            double movementY = -gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;

            //Gets the angle the robot needs to move
            double theta = Math.atan2(movementY, movementX) * 180/Math.PI; //Angle of the gamepad
            double realTheta;
            realTheta = (360-zerodYaw)+theta; //Angle of the robot
            double power = Math.hypot(movementX, movementY);

            //Calulcates x and y values for the robot
            double sin = Math.sin((realTheta * (Math.PI/180)) - (Math.PI / 4));
            double cos = Math.cos((realTheta * (Math.PI/180)) - (Math.PI / 4));
            double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));

            //Calculates the power for each motor
            double motorFLpower = power * cos / maxSinCos + rotation;
            double motorFRpower = power * sin / maxSinCos - rotation;
            double motorBLpower = power * sin / maxSinCos + rotation;
            double motorBRpower = power * cos / maxSinCos - rotation;

            if((power + Math.abs(rotation)) > 1){
                motorFLpower /= power + rotation;
                motorFRpower /= power - rotation;
                motorBLpower /= power + rotation;
                motorBRpower /= power - rotation;
            }

            //Sets power to the motors
            motorFL.setPower(motorFLpower);
            motorFR.setPower(motorFRpower);
            motorBL.setPower(motorBLpower);
            motorBR.setPower(motorBRpower);
        }
    }
}
