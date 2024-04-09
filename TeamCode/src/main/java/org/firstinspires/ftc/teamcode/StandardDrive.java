package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Standard Drive", group="Drive")
public class StandardDrive extends OpMode{

    //Declare Drivetrain Variables
    public DcMotor motorFL;
    public DcMotor motorFR;
    public DcMotor motorBL;
    public DcMotor motorBR;

    @Override
    public void init() {
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
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop() {
        // Gamepad inputs
        double y = -gamepad1.left_stick_y; // Reverse the y-axis (if needed)
        double x = -gamepad1.right_stick_x * 1.1; //Counteracts imperfect strafing
        double rotation = -gamepad1.left_stick_x;

        // Calculate power for each motor
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation), 1);
        double frontLeftPower = (y + x + rotation) / denominator;
        double backLeftPower = (y - x + rotation) / denominator;
        double frontRightPower = (y - x - rotation) / denominator;
        double backRightPower = (y + x - rotation) / denominator;

        //Sets motor powers
        motorFL.setPower(frontLeftPower);
        motorBL.setPower(backLeftPower);
        motorFR.setPower(frontRightPower);
        motorBR.setPower(backRightPower);
    }

    @Override
    public void stop() {}
}
