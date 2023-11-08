package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Autonomous Basic")

public class AutonomousFirstVersionNonBackStage extends LinearOpMode {

    DcMotor FrontR;
    DcMotor BackR;
    DcMotor FrontL;
    DcMotor BackL;
    DcMotor ArmLift;
    Servo Claw;

    static final double Motor_Tick_Count = 1120;


        @Override
        public void runOpMode() throws InterruptedException{

            FrontR = hardwareMap.get(DcMotor.class,"right_front_drive");
            BackR = hardwareMap.get(DcMotor.class,"right_back_drive");
            FrontL = hardwareMap.get(DcMotor.class,"left_front_drive");
            BackL = hardwareMap.get(DcMotor.class,"left_back_drive");
            BackL = hardwareMap.get(DcMotor.class,"left_back_drive");
            ArmLift = hardwareMap.get(DcMotor.class,"arm_lift");
            Claw = hardwareMap.get(Servo.class,"Claw");

            telemetry.addData("ftc", "first");
            telemetry.addData("Init", "is a success");

            FrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackR.setDirection(DcMotorSimple.Direction.REVERSE);

            waitForStart();


            EncoderForward((int)Motor_Tick_Count, .25);


        }

        private void EncoderForward(double target, double speed){

            FrontL.setTargetPosition((int)target);
            FrontR.setTargetPosition((int)target);
            BackL.setTargetPosition((int)target);
            BackR.setTargetPosition((int)target);

            FrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FrontL.setPower(speed);
            FrontR.setPower(speed);
            BackL.setPower(speed);
            BackR.setPower(speed);

            while(opModeIsActive() && FrontL.isBusy() && FrontR.isBusy() && BackL.isBusy() && BackR.isBusy()){
                idle();
            }
        }

}
