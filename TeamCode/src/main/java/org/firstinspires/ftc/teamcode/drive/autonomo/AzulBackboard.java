package org.firstinspires.ftc.teamcode.drive.autonomo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.teleop.SavePose;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous(name = "Azul")
public class AzulBackboard extends LinearOpMode {
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d start_pose = new Pose2d(12.09, 61.31, Math.toRadians(270.00));
        drive.setPoseEstimate(start_pose);

        // Medio y estacionar
//        TrajectorySequence uno = drive.trajectorySequenceBuilder(new Pose2d())
//                .forward(30)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setServoBase(.2)) // Lower servo
//                .waitSeconds(3)
//                .UNSTABLE_addTemporalMarkerOffset(-1, () -> drive.setServoBase(1)) // Raise servo
//                .back(30)
//                .turn(Math.toRadians(90))
//                .build();

        TrajectorySequence Azul_Derecha = drive.trajectorySequenceBuilder(start_pose)
                .lineTo(new Vector2d(12.96, 28.01))
                .turn(Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setServoBase(.2)) // Lower servo
                .waitSeconds(3)
                .setReversed(true)
                .lineTo(new Vector2d(49.40, 29.78))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setServoBase(.2))
                .build();

        TrajectorySequence Azul_Medio = drive.trajectorySequenceBuilder(start_pose)
                .lineTo(new Vector2d(12.96, 28.01))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setServoBase(.2)) // Lower servo
                .waitSeconds(3)
                .strafeTo(new Vector2d(49.40, 29.78))
                .turn(Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setServoBase(.2))
                .build();

        TrajectorySequence Azul_Izquierda = drive.trajectorySequenceBuilder(start_pose)
                .splineTo(new Vector2d(25.23, 45.02), Math.toRadians(-60.10))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> drive.setServoBase(.2)) // Lower servo
                .waitSeconds(3)
                .splineTo(new Vector2d(44.85, 40.47), Math.toRadians(4.24))
                .turn(Math.toRadians(180))
                .build();

deefeefefeef
        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(Azul_Izquierda);

        SavePose.currentPose = drive.getPoseEstimate();
    }
}
