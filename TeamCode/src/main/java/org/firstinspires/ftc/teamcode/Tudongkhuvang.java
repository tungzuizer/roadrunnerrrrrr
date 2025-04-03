package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;



@Autonomous(name = "TUDONGCTÉt")
public class Tudongkhuvang extends LinearOpMode {
    public class Lift {
        private DcMotorEx linerLeft;
        private DcMotorEx linerRight;

        public Lift(HardwareMap hardwareMap) {
            linerLeft = hardwareMap.get(DcMotorEx.class, "linerLeft");
            linerRight = hardwareMap.get(DcMotorEx.class, "linerRight");
            linerLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            linerRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            linerLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            linerRight.setDirection(DcMotorSimple.Direction.REVERSE);


        }
        public class Keonew implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                linerLeft.setTargetPosition(1100);
                linerRight.setTargetPosition(1100 );

                linerLeft.setPower(1);
                linerRight.setPower(1);
                linerLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linerRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return false;
            }
        }
        public Action keonew() {
            return new Keonew();
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linerRight.setPower(1);
                    linerLeft.setPower(1);
                    initialized = true;
                }

                double pos1 = linerLeft.getCurrentPosition();
                double pos2 = linerRight.getCurrentPosition();

                if (pos1 < 1140 && pos2 < 1140) {
                    return true;
                } else {
                    linerLeft.setPower(0);
                    linerRight.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftTrungbinh implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linerRight.setPower(1);
                    linerLeft.setPower(1);
                    initialized = true;
                }

                double pos1 = linerLeft.getCurrentPosition();
                double pos2 = linerRight.getCurrentPosition();

                if (pos1 < 1100 && pos2 < 1100  ) {
                    return true;
                } else {
                    linerLeft.setPower(0);
                    linerRight.setPower(0);
                    return false;
                }
            }
        }
        public Action liftTrungbinh() {
            return new LiftTrungbinh();
        }

        // Hành động hạ xuống (về vị trí 0)
        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linerRight.setPower(-1);
                    linerLeft.setPower(-1);
                    initialized = true;
                }

                double pos1 = linerLeft.getCurrentPosition();
                double pos2 = linerRight.getCurrentPosition();

                if (pos1 > 0 && pos2 > 0) {
                    return true;
                } else {
                    linerLeft.setPower(0);
                    linerRight.setPower(0);
                    return false;
                }
            }
        }

        // Trả về hành động nâng lên

        // Trả về hành động hạ xuống
        public Action liftDown() {
            return new LiftDown();
        }


        public class VevitriDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linerRight.setPower(-1);
                    linerLeft.setPower(-1);
                    initialized = true;
                }

                double pos1 = linerLeft.getCurrentPosition();
                double pos2 = linerRight.getCurrentPosition();

                if (pos1 > 0 && pos2 > 0) {
                    return true;
                } else {
                    linerLeft.setPower(0);
                    linerRight.setPower(0);
                    return false;
                }
            }
        }

        // Trả về hành động nâng lên

        // Trả về hành động hạ xuống
        public Action vevitriDown() {
            return new VevitriDown();
        }

    }

    public class Nang {
        private DcMotorEx motor;
        private DcMotorEx motor1;
        double cang = 0;

        public Nang(HardwareMap hardwareMap) {
            motor = hardwareMap.get(DcMotorEx.class, "motor");
            motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor1.setDirection(DcMotorSimple.Direction.FORWARD);



        }

        public class NangUp implements Action {
            private boolean initialized1 = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized1) {
                    motor.setPower(1);
                    motor1.setPower(1);
                    initialized1 = true;
                }

                double pos3 = motor.getCurrentPosition();
                double pos4 = motor1.getCurrentPosition();
                motor.setTargetPosition(725);
                motor1.setTargetPosition(725 );

                motor.setPower(1);
                motor1.setPower(1);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (pos3 < 735 && pos4 < 735) {
                    return true;
                } else {
                    motor.setTargetPosition(725);
                    motor1.setTargetPosition(725 );

                    motor.setPower(1);
                    motor1.setPower(1);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    return false;
                }
            }
        }
        public Action nangUp() {
            return new NangUp();
        }
        // Hành động hạ xuống (về vị trí 0)

        public class Nangnew implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                motor.setTargetPosition(725);
                motor1.setTargetPosition(725 );

                motor.setPower(1);
                motor1.setPower(1);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return false;
            }
        }
        public Action nangnew() {
            return new Nangnew();
        }



        public class NangDown implements Action {
            private boolean initialized1 = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized1) {

                    initialized1 = true;
                }

                double pos3 = motor.getCurrentPosition();
                double pos4 = motor1.getCurrentPosition();
                motor.setTargetPosition(0);
                motor1.setTargetPosition(0 );
                motor.setPower(0.8);
                motor1.setPower(0.8);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (pos3 > 0 && pos4 > 0) {
                    return true;
                } else {
                    motor.setTargetPosition(0);
                    motor1.setTargetPosition(0 );

                    motor.setPower(0.8);
                    motor1.setPower(0.8);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    return false;
                }
            }
        }

        // Trả về hành động nâng lên

        // Trả về hành động hạ xuống
        public Action nangDown() {
            return new NangDown();
        }


    }


    public class Claw {
        private Servo Servo0, Servo1, Servo2,Servo3,Servo4,Servo5;

        public Claw(HardwareMap hardwareMap) {
            Servo4 = hardwareMap.get(Servo.class, "Servo4");
            Servo5 = hardwareMap.get(Servo.class, "Servo5");
            Servo0 = hardwareMap.get(Servo.class, "Servo0");

            Servo3 = hardwareMap.get(Servo.class, "Servo3");
            Servo2 = hardwareMap.get(Servo.class, "Servo2");
            Servo1 = hardwareMap.get(Servo.class, "Servo1");
            Servo3.setDirection(Servo.Direction.REVERSE);

            Servo1.setDirection(Servo.Direction.FORWARD);
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {


                Servo0.setPosition(0.38);
                sleep(400);
                Servo3.setPosition(0.43);
                Servo4.setPosition(0.43);
                Servo2.setPosition(0);
                Servo1.setPosition(1);

                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {


                Servo3.setPosition(0.99);
                Servo4.setPosition(0.99);
                Servo0.setPosition(0.7);
                Servo2.setPosition(0.4);
                Servo1.setPosition(1);


                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }
        public class GiuClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Servo0.setPosition(0.65);
                Servo3.setPosition(0.35);
                Servo4.setPosition(0.35);
                Servo2.setPosition(0.1);

                return false;
            }
        }

        public Action giuClaw() {
            return new GiuClaw();
        }

        public class HealingClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Servo3.setPosition(0.45);
                Servo4.setPosition(0.45);
                Servo2.setPosition(0.4);

                return false;
            }
        }
        public Action healingClaw() {
            return new HealingClaw();
        }


        public class ChoClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sleep(200);


                return false;
            }
        }
        public Action choClaw() {
            return new ChoClaw();
        }

        public class BaoveClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Servo3.setPosition(0.1);
                Servo4.setPosition(0.1);
                Servo2.setPosition(0.65);

                Servo0.setPosition(0.65);


                return false;
            }
        }
        public Action baoveClaw() {
            return new BaoveClaw();
        }

        public class Gapdoc implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                Servo3.setPosition(0.4);
                Servo4.setPosition(0.4);

                sleep(200);


                Servo2.setPosition(0.1);
                Servo1.setPosition(0.45);

                sleep(200);
                Servo3.setPosition(0.08);
                Servo4.setPosition(0.08);
                Servo2.setPosition(0.25);
                sleep(400);
                Servo0.setPosition(0.1);
                sleep(400);

                Servo1.setPosition(1);
                Servo2.setPosition(0.4);
                Servo3.setPosition(0.4);
                Servo4.setPosition(0.4);
                return false;
            }
        }
        public Action gapdoc() {
            return new Gapdoc();
        }
    }
    private PinpointDrive drivetrain;
    private Lift lift;
    private Claw claw;
    private Nang nang;

    @Override
    public void runOpMode() {
        drivetrain = new PinpointDrive(hardwareMap, new Pose2d(-24, 62, Math.toRadians(270)));
        lift = new Lift(hardwareMap);
        claw = new Claw(hardwareMap);
        nang = new Nang(hardwareMap);

        waitForStart();


        drivetrain.updatePoseEstimate();
        Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose)

                .strafeToLinearHeading(new Vector2d(-37,40),Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-37,30),Math.toRadians(270))

                .splineToConstantHeading(new Vector2d(-45,10),Math.toRadians(200))
                .strafeToLinearHeading(new Vector2d(-45,51),Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-38,35),Math.toRadians(270))

                .splineToConstantHeading(new Vector2d(-53,10),Math.toRadians(200))

                .strafeToLinearHeading(new Vector2d(-53,40),Math.toRadians(270))







                .build());
        Actions.runBlocking(claw.openClaw());

        Actions.runBlocking(nang.nangUp());
        drivetrain.updatePoseEstimate();
        Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose)
                .strafeToLinearHeading(new Vector2d(-50,55.5),Math.toRadians(270))
                .build());
        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(lift.keonew());

        drivetrain.updatePoseEstimate();
        Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose)
                .strafeToLinearHeading(new Vector2d(7,30),Math.toRadians(270))
                .build());
        Actions.runBlocking(lift.liftUp());

        Actions.runBlocking(claw.openClaw());
        Actions.runBlocking(lift.liftDown());
        ////////////////////////////////////////////////////////////////////////////////////////////////////


        drivetrain.updatePoseEstimate();
        Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose)
                .strafeToLinearHeading(new Vector2d(-38,55.3),Math.toRadians(270))
                .build());
        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(lift.keonew());


        drivetrain.updatePoseEstimate();
        Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose)
                .strafeToLinearHeading(new Vector2d(7,30),Math.toRadians(270))
                .build());
        Actions.runBlocking(lift.liftUp());

        Actions.runBlocking(claw.openClaw());
        Actions.runBlocking(lift.liftDown());

////////////////////////////////////////////////////////////////////////////////////////////////////


        drivetrain.updatePoseEstimate();
        Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose)
                .strafeToLinearHeading(new Vector2d(-38,55.2),Math.toRadians(270))
                .build());
        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(lift.liftTrungbinh());
        drivetrain.updatePoseEstimate();
        Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose)
                .strafeToLinearHeading(new Vector2d(7,30),Math.toRadians(270))
                .build());


        Actions.runBlocking(lift.liftUp());
        Actions.runBlocking(claw.openClaw());
        Actions.runBlocking(lift.liftDown());
        ////////////////////////////////////////////////////////////////////////////////////////////////////


        drivetrain.updatePoseEstimate();
        Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose)
                .strafeToLinearHeading(new Vector2d(-38,55.2),Math.toRadians(270))
                .build());
        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(lift.liftTrungbinh());
        drivetrain.updatePoseEstimate();
        Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose)
                .strafeToLinearHeading(new Vector2d(7,30),Math.toRadians(270))
                .build());


        Actions.runBlocking(lift.liftUp());
        Actions.runBlocking(claw.openClaw());
        Actions.runBlocking(lift.liftDown());


        ////////////////////////////////////////////////////////////////////////////////////////////////////

        Actions.runBlocking(nang.nangDown());


    }
}