package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="dieukhinen", group="Linear OpMode")

public class DIEUKHIEN extends LinearOpMode {

    private Servo Servo0, Servo1, Servo2,Servo3,Servo4,Servo5;

    // Declare OpMode members for each of the 4 motors.

    private PIDController motorleft ;
    private PIDController motorright ;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor motor = null;
    private DcMotor motor1 = null;

    private DcMotor linerLeft = null;
    private DcMotor linerRight = null;

    boolean isStateOne = true; // Trạng thái ban đầu
    boolean wasPressed = false; // Kiểm tra trạng thái nhấn nút trước đó


    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    int targetPositionLeft = 0;
    int targetPositionRight = 0;

    private static final int EXTEND_POS = 5000; //tune lai
    private int curPos;
    private static final double SPEED = 1.0;
    double speed = 0.8;
    double gioihan = 0;
    double gioihanmax = 1650;
    double targetPos = 0;
    double hotropin = 720;
    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        leftBack  = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        motor  = hardwareMap.get(DcMotor.class, "motor");
        linerLeft = hardwareMap.get(DcMotor.class, "linerLeft");
        motor1  = hardwareMap.get(DcMotor.class, "motor1");




        motorleft = new PIDController(0.1,0,0.0001);
        motorright = new PIDController(0.1,0,0.0001);



        linerRight = hardwareMap.get(DcMotor.class, "linerRight");
        linerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linerRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linerLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        targetPositionLeft = linerLeft.getCurrentPosition();
        targetPositionRight = linerRight.getCurrentPosition();




        Servo4 = hardwareMap.get(Servo.class, "Servo4");
        Servo5 = hardwareMap.get(Servo.class, "Servo5");
        Servo0 = hardwareMap.get(Servo.class, "Servo0");

        Servo3 = hardwareMap.get(Servo.class, "Servo3");
        Servo2 = hardwareMap.get(Servo.class, "Servo2");
        Servo1 = hardwareMap.get(Servo.class, "Servo1");


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        linerRight.setDirection(DcMotor.Direction.REVERSE);
        linerLeft.setDirection(DcMotor.Direction.FORWARD);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor1.setDirection(DcMotor.Direction.FORWARD);

        Servo3.setDirection(Servo.Direction.REVERSE);

        Servo1.setDirection(Servo.Direction.REVERSE);


        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linerRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linerLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = gamepad1.left_stick_y * speed ;  // Note: pushing stick forward gives negative value
            double lateral = - gamepad1.left_stick_x * speed * 0.6;
            double yaw     =  gamepad1.right_stick_x * speed ;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }



            // Send calculated power to wheels
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("linerLeft: ", linerLeft.getCurrentPosition());
            telemetry.addData("linerRight: ", linerRight.getCurrentPosition());

            telemetry.addData("linernang ", motor.getCurrentPosition());
            encoderkeocang();
            telemetry.update();


            double leftnang ;
            double rightnang ;
            double nang ;

            double drive2= -gamepad2.right_stick_y  ;
            leftnang = Range.clip(drive2,-1, 1) ;
            rightnang  = Range.clip(drive2, -1, 1) ;
  /*       if (drive2 > 0) {
          encoder(2320);
         // encodernang(1);
         }
        if (drive2 < 0) {
             encoder(50);
               //encoderha(1);

        } */
            double drive= -gamepad2.left_stick_y * 1 ;
            nang = Range.clip(drive, -1, 0.4) ;



            if (drive > 0.2) {
                // encoder(1);
                targetPos = 1000;
            }

            if (drive < -0.2) {
                Servo3.setPosition(0.1);
                Servo4.setPosition(0.1);
                Servo2.setPosition(0.65);

                Servo0.setPosition(0.65);
                targetPos = 0;
            }

            encodernangnew(targetPos);


            double   a0 = gamepad2.left_trigger;
            double  a1 = gamepad2.right_trigger;
            double a2 =gamepad1.left_trigger;
            double  a3 = gamepad1.right_trigger;


            if (a3 != 0) {
                speed = 1;

            }else {speed = 0.8 ;}

            if (a2 != 0) {
                speed = 0.3;

            }else {speed = 0.8 ;}




            if (a1 != 0) {

                Servo2.setPosition(0.9);


                sleep(200);
                Servo0.setPosition(0.65);

            }
            if (gamepad1.right_bumper) {
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                targetPos = 735;

                Servo0.setPosition(0.38);
                sleep(400);
                Servo3.setPosition(0.43);
                Servo4.setPosition(0.43);
                Servo2.setPosition(0);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                Servo1.setPosition(1);


            }

            if (gamepad1.left_bumper) {
                Servo3.setPosition(0.99);
                Servo4.setPosition(0.99);
                Servo0.setPosition(0.7);
                Servo2.setPosition(0.4);
                Servo1.setPosition(1);
                targetPos = 735;


            }

            if (gamepad2.right_bumper) {

                Servo3.setPosition(0.0);
                Servo4.setPosition(0.0);
                Servo2.setPosition(0.2);
                sleep(200);
                Servo0.setPosition(0.38);
                Servo1.setPosition(1);
                Servo2.setPosition(0.6);


            }
            if (gamepad2.left_bumper) {
                Servo3.setPosition(0.3);
                Servo4.setPosition(0.3);
                Servo2.setPosition(0.1);
                Servo0.setPosition(0.65);



            }
            if (gamepad1.dpad_right) {
                gioihan = 0;
                gioihanmax  = 1200;
            }
            if (gamepad1.dpad_left) {
                gioihan = 170;
                gioihanmax  = 1610;

            }
            if (gamepad1.dpad_up) {
                hotropin += 3;
            }
            if (gamepad1.dpad_down) {
                hotropin -= 3;

            }
            if (gamepad2.x) {
                Servo1.setPosition(1);


            }
            if (gamepad2.y) {
                Servo1.setPosition(0.45);
                Servo3.setPosition(0.3);
                Servo4.setPosition(0.3);
                Servo2.setPosition(0);
                Servo0.setPosition(0.65);

            }



            boolean isPressed = gamepad1.a; // Kiểm tra xem nút A có đang được bấm không

            if (isPressed && !wasPressed) { // Chỉ thực hiện khi nút A mới được bấm xuống
                isStateOne = !isStateOne; // Chuyển đổi trạng thái
                telemetry.addData("Trạng thái", isStateOne ? "1" : "2");
                telemetry.update();
            }

            wasPressed = isPressed; // Lưu trạng thái của nút A cho lần kiểm tra tiếp theo




        }}
    /*
        public void encoder(double turnage){
           linerLeft.setTargetPosition((int)turnage);
           linerRight.setTargetPosition((int)turnage);
           linerLeft.setPower(1);
           linerRight.setPower(1);

           linerLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           linerRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

       }
       */
    public void encodernang(double nang){
        motor.setTargetPosition(990 * (int)nang );
        motor.setPower(0.7);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void encodernangnew(double nang1){
        motor1.setPower(motorright.control(1 * (int)nang1,motor1.getCurrentPosition()));
        motor.setPower(motorleft.control(1 * (int)nang1,motor.getCurrentPosition()));
    }
    public void encoderha(double nang){
        motor.setTargetPosition(20 * (int)nang );
        motor.setPower(0.3);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void initt() {
        // Lấy vị trí ban đầu của động cơ
        targetPositionLeft = linerLeft.getCurrentPosition();
        targetPositionRight = linerRight.getCurrentPosition();

        // Đặt chế độ encoder để giữ vị trí
        linerLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linerRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void encoderkeocang(){
        double movement = -gamepad2.right_stick_y; // Đảo chiều để điều khiển đúng hướng
        int increment = (int) (movement * 50); // Điều chỉnh độ nhạy

        // Cập nhật target position nhưng giữ trong giới hạn [20, 2320]
        targetPositionLeft = Math.max((int)gioihan, Math.min((int)gioihanmax, targetPositionLeft + increment));
        targetPositionRight = Math.max((int)gioihan, Math.min((int)gioihanmax, targetPositionRight + increment));

        // Cập nhật vị trí cho động cơ
        linerLeft.setTargetPosition(targetPositionLeft);
        linerRight.setTargetPosition(targetPositionRight);

        linerLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linerRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        linerLeft.setPower(1);
        linerRight.setPower(1);





    }

}