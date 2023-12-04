package org.firstinspires.ftc.teamcode.drive.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Servo")
public class servo_test extends LinearOpMode  {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private CRServo intake;
    private DcMotor motor;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        intake  = hardwareMap.get(CRServo.class, "intake");
        motor = hardwareMap.get(DcMotor.class, "motor");




        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        // UpLeft.setDirection(DcMotor.Direction.REVERSE);
        // DownLeft.setDirection(DcMotor.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double ServoPower = 0;
            double motorPower = 0;


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double acelerar = gamepad1.left_stick_y;
            double art = gamepad1.right_stick_y;
            ServoPower   = Range.clip(acelerar , -1.0, 1.0) ;
            motorPower   = Range.clip(art , -1.0, 1.0) ;

            if (gamepad1.dpad_down) {
                motor.setPower(-.4);
            }
            if (gamepad1.dpad_left) {
                intake.setPower(-.4);
            }
            if (gamepad1.dpad_right) {
                intake.setPower(.4);
            }
            if (gamepad1.dpad_up) {
                motor.setPower(.4);
            }


            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;
            // Send calculated power to wheels
            intake.setPower(ServoPower);
            motor.setPower(motorPower);

            // Show the elapsed game time and wheel power.

            telemetry.update();
        }
    }
}
