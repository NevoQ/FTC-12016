package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class AutonomousDrive extends LinearOpMode {

    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor rightRear;

    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setPower(1);
        leftRear.setPower(1);
        rightRear.setPower(1);
        rightFront.setPower(1);

        wait(2000);

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);

    }

}
