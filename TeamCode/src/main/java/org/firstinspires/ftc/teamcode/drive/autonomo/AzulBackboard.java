package org.firstinspires.ftc.teamcode.drive.autonomo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.teleop.SavePose;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous(name = "Adelante")
public class AzulBackboard extends LinearOpMode {
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d start_pose = new Pose2d(0, 0, Math.toRadians(90));
        drive.setPoseEstimate(start_pose);



        // Medio y estacionar
        TrajectorySequence uno = drive.trajectorySequenceBuilder(start_pose)
                .forward(30)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(.2)) // Lower servo
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(0))
                .waitSeconds(.01)
                .build();

//        TrajectorySequence Azul_Derecha = drive.trajectorySequenceBuilder(start_pose)
//                .lineTo(new Vector2d(12.96, 28.01))
//                .turn(Math.toRadians(-90))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(-.2)) // Lower servo
//                .waitSeconds(2)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(0))
//                .setReversed(true)
//                .lineTo(new Vector2d(49.40, 29.78))
//                .UNSTABLE_addTemporalMarkerOffset(0, ()->drive.getPID(100))
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> drive.setServoCaja(.4))
//                .UNSTABLE_addTemporalMarkerOffset(2, ()->drive.getPID(0))
//                .waitSeconds(7)
//                .build();
//
//        TrajectorySequence Azul_Medio = drive.trajectorySequenceBuilder(start_pose)
//                .lineTo(new Vector2d(12.96, 28.01))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(-.2)) // Lower servo
//                .waitSeconds(2)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(0))
//                .strafeTo(new Vector2d(49.40, 29.78))
//                .turn(Math.toRadians(-90))
//                .UNSTABLE_addTemporalMarkerOffset(0, ()->drive.getPID(100))
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> drive.setServoCaja(.4))
//                .UNSTABLE_addTemporalMarkerOffset(2, ()->drive.getPID(0))
//                .waitSeconds(7)
//                .build();
//
//        TrajectorySequence Azul_Izquierda = drive.trajectorySequenceBuilder(start_pose)
//                .splineTo(new Vector2d(25.23, 45.02), Math.toRadians(-60.10))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(-.2)) // Lower servo
//                .waitSeconds(2)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setIntakePower(0))
//                .splineTo(new Vector2d(44.85, 40.47), Math.toRadians(4.24))
//                .turn(Math.toRadians(180))
//                .UNSTABLE_addTemporalMarkerOffset(0, ()->drive.getPID(100))
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> drive.setServoCaja(.4))
//                .UNSTABLE_addTemporalMarkerOffset(2, ()->drive.getPID(0))
//                .waitSeconds(6)
//                .build();
//
//        TrajectorySequence af = drive.trajectorySequenceBuilder(start_pose)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.getPID(100))
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> drive.setServoCaja(.4))
//                .UNSTABLE_addTemporalMarkerOffset(3, ()->drive.getPID(0))
//                .waitSeconds(6)
//                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.setServoBase(.26);

//        if(x<213)
//        {
//            drive.followTrajectorySequence(Azul_Izquierda);
//        }
//        if(x>426)
//        {
//            drive.followTrajectorySequence(Azul_Derecha);
//        }
//        else
//        {
//            drive.followTrajectorySequence(Azul_Medio);
//        }

        drive.followTrajectorySequence(uno);

        SavePose.currentPose = drive.getPoseEstimate();
    }
}
