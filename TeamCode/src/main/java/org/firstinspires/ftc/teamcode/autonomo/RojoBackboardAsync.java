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
        SEGUIR,
        DEJAR_SERVO,
        WAIT_1,
        ELEVADOR_BAJAR,
        ESTACIONAR,
        WAIT_2, IDLE,
        WAIT_3,
        SEPARAR_BACK,
        WAIT_4
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

        double waitTime1 = 1;
        ElapsedTime waitTimer1 = new ElapsedTime();

        TrajectorySequence Rojo_Izquierda = drive.trajectorySequenceBuilder(start_pose)
                .lineTo(new Vector2d(16, -48.06))
                .turn(Math.toRadians(90))
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setServoAutonomo(.7))
                .waitSeconds(.5)
                .build();

        TrajectorySequence Rojo_Izquierda_1 = drive.trajectorySequenceBuilder(Rojo_Izquierda.end())
                .setReversed(true)
                .splineTo(new Vector2d(52.6, -31.5), Math.toRadians(0))
                .build();


        TrajectorySequence Rojo_Medio = drive.trajectorySequenceBuilder(start_pose)
                .lineTo(new Vector2d(11.25, -36.06))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setServoAutonomo(.7))
                .waitSeconds(.5)
                .back(3)
                .build();

        TrajectorySequence Rojo_Medio_1 = drive.trajectorySequenceBuilder(Rojo_Medio.end())
                .setReversed(true)
                .splineTo(new Vector2d(52.6,-41), Math.toRadians(0))
                .build();

        TrajectorySequence Rojo_Derecha = drive.trajectorySequenceBuilder(start_pose)
                .splineTo(new Vector2d(16.81, -48.67), Math.toRadians(87.52))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setServoAutonomo(.7))
                .waitSeconds(.5)
                .build();

        TrajectorySequence Rojo_Derecha_1 = drive.trajectorySequenceBuilder(Rojo_Derecha.end())
                .setReversed(true)
                .splineTo(new Vector2d(52 , -46.3), Math.toRadians(0))
                .build();

        TrajectorySequence Rojo_separar_izquierda = drive.trajectorySequenceBuilder(Rojo_Izquierda_1.end())
                .forward(10)
                .build();

        TrajectorySequence Rojo_separar_medio = drive.trajectorySequenceBuilder(Rojo_Medio_1.end())
                .forward(10)
                .build();

        TrajectorySequence Rojo_separar_derecha = drive.trajectorySequenceBuilder(Rojo_Derecha_1.end())
                .forward(10)
                .build();

        TrajectorySequence Rojo_Final_Izq = drive.trajectorySequenceBuilder(Rojo_separar_izquierda.end())
                .strafeTo(new Vector2d(45, -67))
                .setReversed(true)
                .lineTo(new Vector2d(61.88, -67))
                .build();

        TrajectorySequence Rojo_Final_Mid = drive.trajectorySequenceBuilder(Rojo_separar_medio.end())
                .strafeTo(new Vector2d(45, -67))
                .setReversed(true)
                .lineTo(new Vector2d(61.88, -67))
                .build();

        TrajectorySequence Rojo_Final_Der = drive.trajectorySequenceBuilder(Rojo_separar_derecha.end())
                .strafeTo(new Vector2d(45, -67))
                .setReversed(true)
                .lineTo(new Vector2d(61.88, -67))
                .build();




        while (!isStarted()) {
            telemetry.addData("POSICION: ", redElement.getAnalysis());
            telemetry.update();
        }

        drive.setServoAutonomo(.07);
        waitForStart();

        if (isStopRequested()) return;
        int y;



        currentState = State.TRAJECTORY_1;
        if (redElement.getAnalysis()==1) { drive.followTrajectorySequenceAsync(Rojo_Medio); y=1;}
        else if(redElement.getAnalysis()==2) { drive.followTrajectorySequenceAsync(Rojo_Derecha); y=2; }
        else { drive.followTrajectorySequenceAsync(Rojo_Izquierda); y=3;}

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case TRAJECTORY_1:
                    drive.setServoBase(.96);
                    drive.setServoCaja(.4);
                    if (!drive.isBusy()) {
                        currentState = State.ELEVADOR_SUBIR;
                    }
                    break;
                case ELEVADOR_SUBIR:
                    target = 1400;
                    if (!drive.isBusy()) {
                        currentState = State.SEGUIR;
                        if(y==1){drive.followTrajectorySequenceAsync(Rojo_Medio_1);}
                        if(y==2){drive.followTrajectorySequenceAsync(Rojo_Derecha_1);}
                        if(y==3){drive.followTrajectorySequenceAsync(Rojo_Izquierda_1);}
                    }
                    break;
                case SEGUIR:
                    drive.setServoBase(.62);
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
                    drive.setServoCaja(.3);
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_2;
                        waitTimer1.reset();
                    }
                    break;
                case WAIT_2:
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.SEPARAR_BACK;
                        if(y==1){drive.followTrajectorySequenceAsync(Rojo_separar_medio);}
                        if(y==2){drive.followTrajectorySequenceAsync(Rojo_separar_derecha);}
                        if(y==3){drive.followTrajectorySequenceAsync(Rojo_separar_izquierda);}
                    }
                    break;
                case SEPARAR_BACK:
                    if (!drive.isBusy()) {
                        drive.setServoBase(.95);
                        currentState = State.ELEVADOR_BAJAR;
                    }
                    break;
                case ELEVADOR_BAJAR:
                    target = 0;
                    if (!drive.isBusy()) {
                        if(y==1){drive.followTrajectorySequenceAsync(Rojo_Final_Mid);}
                        if(y==2){drive.followTrajectorySequenceAsync(Rojo_Final_Der);}
                        if(y==3){drive.followTrajectorySequenceAsync(Rojo_Final_Izq);}
                        currentState = State.ESTACIONAR;
                    }
                    break;
                case ESTACIONAR:
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
