package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Tank Drive", group="Drive")
public class TankDrive extends OpMode{

    //Declare Drivetrain Variables
    public DcMotor motorFL;
    public DcMotor motorFR;
    public DcMotor motorBL;
    public DcMotor motorBR;

    //Declare movement variables
    public double left;
    public double right;
    public double strafeRight;
    public double strafeLeft;

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
        // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        strafeRight = gamepad1.right_stick_x;
        strafeLeft = gamepad1.left_stick_x;

        //Sets power to all motors
        if (gamepad1.right_stick_x > 0.5) {
            motorFL.setPower(strafeRight);
            motorBL.setPower(-strafeRight);
            motorFR.setPower(-strafeRight);
            motorBR.setPower(strafeRight);
        } else if (gamepad1.left_stick_x < -0.5) {
            motorFL.setPower(strafeLeft);
            motorBL.setPower(-strafeLeft);
            motorFR.setPower(-strafeLeft);
            motorBR.setPower(strafeLeft);
        } else {
            motorFL.setPower(left);
            motorBL.setPower(left);
            motorFR.setPower(right);
            motorBR.setPower(right);
        }
    }

    @Override
    public void stop() {}
}
