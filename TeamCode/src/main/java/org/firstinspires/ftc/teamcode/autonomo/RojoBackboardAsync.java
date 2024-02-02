package org.firstinspires.ftc.teamcode.autonomo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.util.Size;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.vision.RedElement;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.teleop.SavePose;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "Rojo Back")
public class RojoBackboardAsync extends LinearOpMode {
    private OpenCvCamera camera;
    private RedElement redElement;
    private String webcamName = "Webcam 1";
    int target = 0;

    enum State {
        TRAJECTORY_1,
        ELEVADOR_SUBIR,
        DEJAR_SERVO,
        WAIT_1,
        ELEVADOR_BAJAR,
        ESTACIONAR,
        WAIT_2, IDLE
    }

    State currentState = State.IDLE;
    Pose2d start_pose = new Pose2d(15.92, -67.31, Math.toRadians(90.00));

    public void runOpMode() throws InterruptedException {
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName));
        redElement = new RedElement();
        camera.setPipeline(redElement);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start_pose);

        double waitTime1 = 3;
        ElapsedTime waitTimer1 = new ElapsedTime();

        TrajectorySequence Rojo_Izquierda = drive.trajectorySequenceBuilder(start_pose)
                .lineTo(new Vector2d(16, -44.06))
                .turn(Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(0.6)) // Lower servo
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(0))
                .waitSeconds(.01)
                .setReversed(true)
                .splineTo(new Vector2d(52.6, -31.5), Math.toRadians(0))
                .build();


        TrajectorySequence Rojo_Medio = drive.trajectorySequenceBuilder(start_pose)
                .lineTo(new Vector2d(11.25, -38.06))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(.6)) // Lower servo
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(0))
                .waitSeconds(.01)
                .strafeTo(new Vector2d(46.31, -36.00))
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(52.6,-45))
                .build();

        TrajectorySequence Rojo_Derecha = drive.trajectorySequenceBuilder(start_pose)
                .splineTo(new Vector2d(23.81, -53.67), Math.toRadians(87.52))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(.6)) // Lower servo
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(0))
                .waitSeconds(.01)
                .setReversed(true)
                .splineTo(new Vector2d(52 , -48.3), Math.toRadians(0))
                .build();


//        TrajectorySequence Rojo_Final = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .strafeTo(new Vector2d(47, -60))
//                .setReversed(true)
//                .lineTo(new Vector2d(61.88, -61.12))
//                .build();


        while (!isStarted()) {
            telemetry.addData("POSICION: ", redElement.getAnalysis());
            telemetry.update();
        }
        waitForStart();

        if (isStopRequested()) return;


        currentState = RojoBackboardAsync.State.TRAJECTORY_1;
        if (redElement.getAnalysis()==1) { drive.followTrajectorySequenceAsync(Rojo_Medio); }
        else if(redElement.getAnalysis()==2) { drive.followTrajectorySequenceAsync(Rojo_Derecha); }
        else { drive.followTrajectorySequenceAsync(Rojo_Izquierda); }

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case TRAJECTORY_1:
                    drive.setServoBase(.26);
                    drive.setServoCaja(.3);
                    if (!drive.isBusy()) {
                        currentState = State.ELEVADOR_SUBIR;
                    }
                    break;
                case ELEVADOR_SUBIR:
                    target = 1600;
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_1;
                        waitTimer1.reset();
                    }
                    break;
                case WAIT_1:
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.DEJAR_SERVO;

                    }
                    break;
                case DEJAR_SERVO:
                    drive.setServoBase(.38);
                    drive.setServoCaja(.4);
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_2;
                        waitTimer1.reset();
                    }
                    break;
                case WAIT_2:
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.ELEVADOR_BAJAR;
                    }
                    break;

                case ELEVADOR_BAJAR:
                    drive.setServoBase(.26);
                    target = 0;
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }
            drive.getPID(target);
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();


            SavePose.currentPose = drive.getPoseEstimate();
        }
    }


}
