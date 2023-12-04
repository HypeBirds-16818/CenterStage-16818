package org.firstinspires.ftc.teamcode.drive.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "testtop")
public class test_teleoptop extends LinearOpMode {
    private PIDController controller, controllerA;

    public static double p = 0.009, i = 0, d = 0.0001;
    public static double f = 0.14;


    public static double pA = 0.03, iA = 0, dA = 0.0001;
    public static double fA = 0.1;

    public int target = 0;
    public int targetA = 0;

    private final double ticks_in_degree = 751.8;
    private final double ticks_in_degreeA = 1425.1;

    private DcMotorEx top_motor_1;
    private DcMotorEx top_motor_2;
    private DcMotorEx bottom_motor_1;
    private DcMotorEx bottom_motor_2;

    @Override
    public void runOpMode() throws InterruptedException {


        controller = new PIDController(p, i, d);
        controllerA = new PIDController(pA, iA, dA);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        top_motor_1 = hardwareMap.get(DcMotorEx.class, "top_motor_1");
        top_motor_2 = hardwareMap.get(DcMotorEx.class, "top_motor_2");
        top_motor_1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        top_motor_2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bottom_motor_1 = hardwareMap.get(DcMotorEx.class, "bottom_motor_1");
        bottom_motor_2 = hardwareMap.get(DcMotorEx.class, "bottom_motor_2");
        bottom_motor_1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bottom_motor_2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            controller.setPID(p, i, d);

            if (gamepad1.dpad_up) {
                target += 10;
            }
            if (gamepad1.dpad_down) {
                target -= 10;
            }
            if (gamepad1.dpad_right) {
                targetA += 10;
            }
            if (gamepad1.dpad_left) {
                targetA -= 10;
            }

            int basePos = (top_motor_1.getCurrentPosition() + top_motor_2.getCurrentPosition()) / 2;
            double pid = controller.calculate(basePos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            double power = pid + ff;

            int basePosA = (bottom_motor_1.getCurrentPosition() + bottom_motor_2.getCurrentPosition()) / 2;
            double pidA = controllerA.calculate(basePosA, targetA);
            double ffA = Math.cos(Math.toRadians(targetA / ticks_in_degreeA)) * f;
            double powerA = pidA + ffA;

            top_motor_1.setPower(power);
            top_motor_2.setPower(power);

            bottom_motor_1.setPower(powerA);
            bottom_motor_2.setPower(powerA);

            telemetry.addData("base position: ", basePos);
            telemetry.addData("target position: ", target);
            telemetry.addData("base position botton: ", basePosA);
            telemetry.addData("target position botton: ", targetA);
            telemetry.update();

        }
    }

}
