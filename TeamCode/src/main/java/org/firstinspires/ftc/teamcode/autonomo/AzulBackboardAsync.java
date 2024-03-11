package org.firstinspires.ftc.teamcode.autonomo;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.teleop.SavePose;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BlueElement;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Azul Back")
public class AzulBackboardAsync extends LinearOpMode {
    private OpenCvCamera camera;
    private BlueElement blueElement;
    private String webcamName = "Webcam 1";

    int target = 0;
    enum State {
        TRAJECTORY_1,
        ELEVADOR_SUBIR,
        SEGUIR,
        DEJAR_SERVO,
        WAIT_1,
        SEPARAR_BACK,
        ELEVADOR_BAJAR,
        ESTACIONAR,
        WAIT_2, IDLE         // Our bot will enter the IDLE state when done
    }
    State currentState = State.IDLE;
    Pose2d start_pose = new Pose2d(15.92, 67.31, Math.toRadians(270.00));
    public void runOpMode() throws InterruptedException {
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName));
        blueElement = new BlueElement();
        camera.setPipeline(blueElement);

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

        TrajectorySequence Azul_Izquierda = drive.trajectorySequenceBuilder(start_pose)
                .splineTo(new Vector2d(30, 48.67), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setServoAutonomo(.7))
                .waitSeconds(.5)
                .build();

        TrajectorySequence Azul_Izq_1 = drive.trajectorySequenceBuilder(Azul_Izquierda.end())
                .setReversed(true)
                .splineTo(new Vector2d(51 , 50), Math.toRadians(0))
                .build();

        TrajectorySequence Azul_Medio = drive.trajectorySequenceBuilder(start_pose)
                .lineTo(new Vector2d(14.92, 38))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setServoAutonomo(.7))
                .waitSeconds(.5)
                .back(3)
                .build();

        TrajectorySequence Azul_Mid_1 = drive.trajectorySequenceBuilder(Azul_Medio.end())
                .setReversed(true)
                .splineTo(new Vector2d(52 , 42.5), Math.toRadians(0))
                .build();

        TrajectorySequence Azul_Derecha = drive.trajectorySequenceBuilder(start_pose)
                .lineTo(new Vector2d(16, 26     ))
                .turn(Math.toRadians(-90))
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setServoAutonomo(.7))
                .waitSeconds(.5)
                .build();

        TrajectorySequence Azul_Der_1 = drive.trajectorySequenceBuilder(Azul_Derecha.end())
                .setReversed(true)
                .splineTo(new Vector2d(52.6,33.6), Math.toRadians(0))
                .build();

        TrajectorySequence Azul_separar_izquierda = drive.trajectorySequenceBuilder(Azul_Izq_1.end())
                .forward(10)
                .build();

        TrajectorySequence Azul_separar_medio = drive.trajectorySequenceBuilder(Azul_Mid_1.end())
                .forward(10)
                .build();

        TrajectorySequence Azul_separar_derecha = drive.trajectorySequenceBuilder(Azul_Der_1.end())
                .forward(10)
                .build();

        TrajectorySequence Azul_Final_Izq = drive.trajectorySequenceBuilder(Azul_separar_izquierda.end())
                .strafeTo(new Vector2d(45, 65))
                .setReversed(true)
                .lineTo(new Vector2d(61.88, 65))
                .build();

        TrajectorySequence Azul_Final_Mid = drive.trajectorySequenceBuilder(Azul_separar_medio.end())
                .strafeTo(new Vector2d(45, 65))
                .setReversed(true)
                .lineTo(new Vector2d(61.88, 65))
                .build();

        TrajectorySequence Azul_Final_Der = drive.trajectorySequenceBuilder(Azul_separar_derecha.end())
                .strafeTo(new Vector2d(45, 65))
                .setReversed(true)
                .lineTo(new Vector2d(61.88, 65))
                .build();

        while (!isStarted()) {
            telemetry.addData("Posicion: ", blueElement.getAnalysis());
            telemetry.update();
        }


        waitForStart();

        if (isStopRequested()) return;
        int y;
        drive.setServoAutonomo(.07);

        currentState = State.TRAJECTORY_1;
        if (blueElement.getAnalysis()==1) { drive.followTrajectorySequenceAsync(Azul_Izquierda); y=3; }
        else if(blueElement.getAnalysis()==2) { drive.followTrajectorySequenceAsync(Azul_Medio); y=1; }
        else { drive.followTrajectorySequenceAsync(Azul_Derecha); y=2; }

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case TRAJECTORY_1:
                    drive.setServoBase(.95);
                    drive.setServoCaja(.4);
                    if (!drive.isBusy()) {
                        currentState = State.ELEVADOR_SUBIR;
                    }
                    break;
                case ELEVADOR_SUBIR:
                    target = 1200;
                    if (!drive.isBusy()) {
                        currentState = State.SEGUIR;
                        if(y==1){drive.followTrajectorySequenceAsync(Azul_Mid_1);}
                        if(y==2){drive.followTrajectorySequenceAsync(Azul_Der_1);}
                        if(y==3){drive.followTrajectorySequenceAsync(Azul_Izq_1);}

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
                        currentState =State.SEPARAR_BACK;
                        if(y==1){drive.followTrajectorySequenceAsync(Azul_separar_medio);}
                        if(y==2){drive.followTrajectorySequenceAsync(Azul_separar_derecha);}
                        if(y==3){drive.followTrajectorySequenceAsync(Azul_separar_izquierda);}
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
                        currentState = State.ESTACIONAR;
                        if(y==1){drive.followTrajectorySequenceAsync(Azul_Final_Mid);}
                        if(y==2){drive.followTrajectorySequenceAsync(Azul_Final_Der);}
                        if(y==3){drive.followTrajectorySequenceAsync(Azul_Final_Izq);}
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
