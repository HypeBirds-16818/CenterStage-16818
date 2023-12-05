/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.drive.teleop;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name="Cascarita", group="Linear Opmode")

public class Cascarita extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor UpLeft = null, UpRight = null, DownLeft = null, DownRight = null;
    private CRServo caja_1 = null, caja_2 = null, intake = null, base_1 = null, base_2 = null, puerta = null;

    private PIDController controller_t, controller_b;
    private static double pt = 0.009, it = 0, dt = 0.0001, ft = 0.14;
    private static double pb = 0.03, ib = 0, db = 0.0001, fb = 0.1;
    private int target_t = 0, target_b = 0;
    private final double ticks_in_degree_b = 1425.1, ticks_in_degree_t = 751.8;
    private DcMotorEx top_motor_1, top_motor_2;
    private DcMotorEx bottom_motor_1, bottom_motor_2;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        UpLeft  = hardwareMap.get(DcMotor.class, "rightFront");
        UpRight = hardwareMap.get(DcMotor.class, "leftFront");
        DownLeft  = hardwareMap.get(DcMotor.class, "rightRear");
        DownRight  = hardwareMap.get(DcMotor.class, "leftRear");
        caja_1 = hardwareMap.get(CRServo.class, "caja1");
        caja_2 = hardwareMap.get(CRServo.class, "caja2");
        intake = hardwareMap.get(CRServo.class, "intake");
        base_1 = hardwareMap.get(CRServo.class, "base1");
        base_2 = hardwareMap.get(CRServo.class, "base2");
        puerta = hardwareMap.get(CRServo.class, "puerta");
        bottom_motor_1 = hardwareMap.get(DcMotorEx.class, "bottomMotor1");
        bottom_motor_2 = hardwareMap.get(DcMotorEx.class, "bottomMotor2");
        top_motor_1 = hardwareMap.get(DcMotorEx.class, "topMotor1");
        top_motor_1 = hardwareMap.get(DcMotorEx.class, "topMotor2");


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
            double UpLeftPower = 0;
            double UpRightPower = 0;
            double DownLeftPower = 0;
            double DownRightPower = 0;
            double cajaPower = 0;
            double basePower = 0;
            double intakePower = 0;
            double puertaPower = 0;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double acelerar = gamepad1.left_stick_y;
            double der_izq = gamepad1.left_stick_x;
            double turn  =  -gamepad1.right_stick_x;
            UpLeftPower   = Range.clip(acelerar - der_izq + turn, -1.0, 1.0) ;
            UpRightPower   = Range.clip(acelerar + der_izq - turn, -1.0, 1.0) ;
            DownLeftPower = Range.clip(acelerar - der_izq + turn, -1.0, 1.0);
            DownRightPower = Range.clip(acelerar + der_izq - turn, -1.0, 1.0);

            double movimiento_caja = gamepad2.left_stick_y;
            double movimiento_intake_adelante = gamepad2.right_trigger;
            double movimiento_intake_atras = gamepad2.left_trigger;
            double abrir_puerta = gamepad2.right_stick_y;
            cajaPower = Range.clip(movimiento_caja, -.5, .5);
            intakePower = Range.clip(movimiento_intake_adelante - movimiento_intake_atras, -1.0, 1.0);
            puertaPower = Range.clip(abrir_puerta, -.5, .5);



            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;



            if (gamepad1.dpad_down) {
                UpLeft.setPower(-.4);
                UpRight.setPower(-.4);
                DownLeft.setPower(-.4);
                DownRight.setPower(-.4);
            }
            if (gamepad1.dpad_left) {
                UpLeft.setPower(-.4);
                UpRight.setPower(.4);
                DownLeft.setPower(.4);
                DownRight.setPower(-.4);
            }
            if (gamepad1.dpad_right) {
                UpLeft.setPower(.4);
                UpRight.setPower(-.4);
                DownLeft.setPower(-.4);
                DownRight.setPower(.4);
            }
            if (gamepad1.dpad_up) {
                UpLeft.setPower(.4);
                UpRight.setPower(.4);
                DownLeft.setPower(.4);
                DownRight.setPower(.4);
            }
            // Send calculated power to wheels
            UpLeft.setPower(UpLeftPower);
            UpRight.setPower(UpRightPower);
            DownLeft.setPower(DownLeftPower);
            DownRight.setPower(DownRightPower);
            intake.setPower(intakePower);
            caja_1.setPower(cajaPower);
            caja_2.setPower(cajaPower);
            base_1.setPower(basePower);
            base_2.setPower(basePower);
            puerta.setPower(puertaPower);

            // Show the elapsed game time and wheel power.

            telemetry.update();
        }
    }
}