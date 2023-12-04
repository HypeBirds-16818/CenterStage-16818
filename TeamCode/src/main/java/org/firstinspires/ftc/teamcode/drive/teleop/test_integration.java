package org.firstinspires.ftc.teamcode.drive.teleop;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Integrated Test")
public class test_integration extends LinearOpMode {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    CRServo planeServo;


    SampleMecanumDrive.intakeState intakeState = SampleMecanumDrive.intakeState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        drive.setModeIntake(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        planeServo = hardwareMap.get(CRServo.class, "planeServo");
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            setPid(drive, "Top", 100);

            if(gamepad1.a){
                planeServo.setPower(1);
            }
            if(gamepad1.b){
                planeServo.setPower(-1);
            }
            if(gamepad1.x){
                planeServo.setPower(0);
            }

            switch (intakeState) {
                case IDLE:
                    if(gamepad2.a){
                        //Tirar intake abajo
                        intakeState = SampleMecanumDrive.intakeState.TAKING;
                    }
                break;
                case TAKING:
                    //rodar intake x segundos hacia adentro

                    if(gamepad2.a){ //Presionar a cuando agarre las cosas
                        //subir intaka y pararlo
                        intakeState = SampleMecanumDrive.intakeState.RAISING1;
                    }

                break;
                case RAISING1:
                    //subir el intake primer etapa

                    intakeState = SampleMecanumDrive.intakeState.RAISING2;

                    break;
                case RAISING2:

                    //subir el intake etapa final
                    intakeState = SampleMecanumDrive.intakeState.TURNING;

                    break;
                case TURNING:

                    //voltear los servos de la caja para dejarla pegada al backdrop
                    if(gamepad2.a) {  //esperar input para voltear
                        intakeState = SampleMecanumDrive.intakeState.OPENING;
                    }
                    break;
                case OPENING:

                    //abrir servo de caja para dejar caer los pixeles
                    if(gamepad2.a){ //esperar input para empezar descenso
                        intakeState = SampleMecanumDrive.intakeState.TURNINGBACK;
                    }

                    break;
                case TURNINGBACK:

                    //cerrar caja
                    //voltearla a posicion inicial
                    intakeState = SampleMecanumDrive.intakeState.FALLING1;

                    break;
                case FALLING1:

                    //caida etapa inicial
                    intakeState = SampleMecanumDrive.intakeState.FALLING2;

                    break;
                case FALLING2:

                    //caida etapa final (posicion inicial)
                    //bajar motor lentamente para asegurar que este hasta abajo
                    //reiniciar encoders para asegurar el 0
                    intakeState = SampleMecanumDrive.intakeState.IDLE;

                    break;
            }
        }
    }

    public void setPid(SampleMecanumDrive drive, String PIDcase, int target){
        if (PIDcase.equals("Top")){
            drive.getTopPID(target);
        }
        else if (PIDcase.equals("Bottom")){
            drive.getBottomPID(target);
        }



    }

}


