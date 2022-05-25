package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
@Config
public class BlueTeam extends LinearOpMode {

    public static int mountDepositTarget = -600;
    public static int armDepositTarget = 320;

    public static double carouselPower = 1;
    public static boolean turretMode = false;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor turretMount = hardwareMap.dcMotor.get("turretMount");
        DcMotor turretArm = hardwareMap.dcMotor.get("turretArm");
        DcMotor carousel = hardwareMap.dcMotor.get("carousel");
        DcMotor roller = hardwareMap.dcMotor.get("roller");

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMount.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turretArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMount.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            // drive train
            drive.setWeightedDrivePower(
                    new Pose2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x
                            )/*.rotated(-drive.getExternalHeading())*/,
                            -gamepad1.right_stick_x
                    ).div(gamepad1.left_bumper ? 3 : 1)
            );

            drive.update();
            // end of drive train

            //dpad choice

            if(gamepad2.dpad_up) {
                mountDepositTarget = 450;
                turretArm.setTargetPosition(mountDepositTarget);
                turretArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretArm.setPower(0.8);
            } else if(gamepad2.dpad_left) {
                mountDepositTarget = 320;
                turretArm.setTargetPosition(mountDepositTarget);
                turretArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turretArm.setPower(0.8);
            }

            //end of dpad choice

            // turret
            if (gamepad2.a) {
                turretMount.setTargetPosition(mountDepositTarget);
                turretMount.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMount.setPower(0.8);

                turretArm.setTargetPosition(armDepositTarget);
                turretArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretArm.setPower(0.8);

            } else if (gamepad2.b) {

                turretMount.setTargetPosition(0);
                turretMount.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMount.setPower(0.5);

            }

            // TurretMode
            if (gamepad2.x){
                turretMode = !turretMode;
            }

            if (turretMode) {
                if (gamepad2.right_stick_x > 0) {

                    turretMount.setTargetPosition(turretMount.getCurrentPosition() + 100);
                    turretMount.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    turretMount.setPower(0.5);

                }  else if (gamepad2.right_stick_x < 0) {

                    turretMount.setTargetPosition(turretMount.getCurrentPosition() - 100);
                    turretMount.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    turretMount.setPower(0.5);

                }

                if (gamepad2.left_stick_y < -0.15) {

                    turretArm.setTargetPosition(turretArm.getCurrentPosition() + 100);
                    turretArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    turretArm.setPower(0.5);

                }  else if (gamepad2.left_stick_y > 0.15) {

                    turretArm.setTargetPosition(turretArm.getCurrentPosition() - 100);
                    turretArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    turretArm.setPower(0.5);

                }
            }
            // End of TurretMode

            // end of turret

            // carousel
            if (gamepad2.left_bumper)
                carousel.setPower(carouselPower);
            else if (gamepad2.right_bumper)
                carousel.setPower(-carouselPower);
            else carousel.setPower(0);
            // end of carousel

            // roller
            if (gamepad1.right_trigger > 0) {
                roller.setPower(0.7);
            }  else if(gamepad1.left_trigger > 0) {
                roller.setPower(-1);
                turretArm.setTargetPosition(0);
            } else roller.setPower(0);
            // end of roller

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            telemetry.addData("Mount Position", turretMount.getCurrentPosition());
            telemetry.addData("Arm Position", turretArm.getCurrentPosition());

            telemetry.addData("LeftStickY", gamepad2.left_stick_y);
            telemetry.addData("RightStickX", gamepad2.right_stick_x);

            telemetry.addData("Location", armDepositTarget);

            telemetry.addData("TurretMode", turretMode);

            telemetry.update();
        }
    }
}