package org.firstinspires.ftc.teamcode.autonomo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.teleop.SavePose;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.RedElement;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "Rojo HP con Back")
public class RojoHumanPlayerBackboard extends LinearOpMode {
    private OpenCvCamera camera;
    private RedElement redElement;
    private String webcamName = "Webcam 1";
    int target = 0;

    enum State {
        ESPERA_INICIAL,
        TRAJECTORY_1,
        ELEVADOR_SUBIR,
        SEGUIR,
        DEJAR_SERVO,
        WAIT_1,
        ELEVADOR_BAJAR,
        ESTACIONAR,
        SEPARAR_BACK,
        WAIT_2, IDLE
    }

    State currentState = State.IDLE;
    Pose2d start_pose = new Pose2d(-38.61, -67.31, Math.toRadians(90.00));


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
                .lineTo(new Vector2d(-38, -48.06))
                .turn(Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setServoAutonomo(1))
                .waitSeconds(.5)
                .setReversed(true)
                .strafeTo(new Vector2d(-38,-9))
                .lineTo(new Vector2d(41.47, -9.35))
                .build();


        TrajectorySequence Rojo_Izquierda_1 = drive.trajectorySequenceBuilder(Rojo_Izquierda.end())
                .setReversed(false)
                .strafeTo(new Vector2d(52.6, -31.5))
                .build();


        TrajectorySequence Rojo_Medio = drive.trajectorySequenceBuilder(start_pose)
                .splineTo(new Vector2d(-38,-9),Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setServoAutonomo(1))
                .waitSeconds(.5)
                .setReversed(true)
                .lineTo(new Vector2d(41.47, -9.35))
                .build();



        TrajectorySequence Rojo_Medio_1 = drive.trajectorySequenceBuilder(Rojo_Medio.end())
                .setReversed(true)
                .strafeTo(new Vector2d(52.6,-41))
                .build();

        TrajectorySequence Rojo_Derecha = drive.trajectorySequenceBuilder(start_pose)
                .lineTo(new Vector2d(-38, -48.06))
                .turn(Math.toRadians(-90))
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setServoAutonomo(1))
                .waitSeconds(.5)
                .strafeTo(new Vector2d(-38,-9))
                .setReversed(true)
                .lineTo(new Vector2d(41.47, -9.35))
                .build();

        TrajectorySequence Rojo_Derecha_1 = drive.trajectorySequenceBuilder(Rojo_Derecha.end())
                .setReversed(true)
                .strafeTo(new Vector2d(52 , -46.3))
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


        while (!isStarted()) {
            telemetry.addData("POSICION: ", redElement.getAnalysis());
            telemetry.update();
        }
        drive.setServoAutonomo(.07);
        waitForStart();

        if (isStopRequested()) return;
        int y;


        currentState = State.ESPERA_INICIAL;
        if (redElement.getAnalysis()==1) { y=1;}
        else if(redElement.getAnalysis()==2) { y=2; }
        else { y=3;}
        waitTimer1.reset();

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case ESPERA_INICIAL:
                    if (waitTimer1.seconds() >= 12) {
                        if (y==1) { drive.followTrajectorySequenceAsync(Rojo_Medio); }
                        else if(y==2) { drive.followTrajectorySequenceAsync(Rojo_Derecha); }
                        else { drive.followTrajectorySequenceAsync(Rojo_Izquierda); }
                    }
                    break;

                case TRAJECTORY_1:
                    drive.setServoBase(.95);
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
                    drive.setServoCaja(.4);
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
                    drive.setServoBase(.62);
                    target = 0;
                    if (!drive.isBusy()) {
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
