package org.firstinspires.ftc.teamcode.drive.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpMty")
public class TeleOp extends LinearOpMode {
    public static int target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(SavePose.currentPose);

        //drive.setSlideMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        drive.setServoBase(.95);
        drive.setServoCaja(.3);

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {


            // Movimiento de chasis
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );


            //Small movement
            if(gamepad1.dpad_left)
                drive.setMotorPowers(-.2, .2, -.2, .2);
            if(gamepad1.dpad_right)
                drive.setMotorPowers(.2, -.2, .2, -.2);
            if(gamepad1.dpad_up)
                drive.setMotorPowers(.2, .2, .2, .2);
            if(gamepad1.dpad_down)
                drive.setMotorPowers(-.2, -.2, -.2, -.2);



            if(gamepad1.a){
                drive.setServoAvion(0);
            }

            double poderElevador = 0;
            if(gamepad1.right_bumper)
            {
                if(drive.getElevadorPos() < 14800)
                    poderElevador = 1;
                else
                    poderElevador = 0;
            }
            if(gamepad1.left_bumper)
            {
                if(drive.getElevadorPos() > 300)
                    poderElevador = -1;
                else
                    poderElevador=0;
            }
            drive.setElevadorPower(poderElevador);




            //Elevador
            drive.getPID(target);
            if(gamepad2.a){
                //base
                target = 0;
            }
            if(gamepad2.b){
                //segunda linea
                target = 1600;
                //drive.setServoElevador(.7);
            }
            if(gamepad2.y){
                //tercera linea
                target = 2500;
                //drive.setServoElevador(.7);
            }
            if(gamepad2.right_bumper)
                target += 100;
            if(gamepad2.left_bumper)
                target -= 100;

            //Intake
            double intakePower = Range.clip(gamepad2.right_stick_y, -1, 1);
            drive.setIntakePower(intakePower);


            //Outake
            //Poner en posicion de subida
            if(gamepad2.dpad_up) {
                drive.setServoCaja(.3);
                drive.setServoBase(.95);
            }
            //Poner en posicion de bajada, faltan valores reales
            if(gamepad2.dpad_down) {
                drive.setServoBase(.62);
                drive.setServoCaja(.4);
            }
            if(gamepad2.dpad_left)
            {
                drive.setServoCaja(.4);
            }
            if(gamepad2.dpad_right)
            {
                drive.setServoBase(.35);
            }


            //Climber (to do)

            // Update everything. Odometry. Etc.
            drive.update();

        }
    }
}