package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="C1rcu1t", group="Scripts")
public class C1rcu1t extends LinearOpMode {

    private DcMotor MotorL1;
    private DcMotor MotorR1;
    private DcMotor MotorL2;
    private DcMotor MotorR2;

    int[] TowerTags = {4, 5,}; 
    int[] Motif1ID = {1};
    int[] Motif2ID = {2};
    int[] Motif3ID = {3};
    
    double hue1;
    double hue2;
    double hue3;

    int order = 0;
    
   
    boolean isTrackingEnabled = false;
    boolean lastLeftBumperState = false;
    
    private HuskyLens huskyLens;
    
    private NormalizedColorSensor LeftClr;
    private NormalizedColorSensor RightClr;
    private NormalizedColorSensor BackClr;
    
    private ElapsedTime runtime = new ElapsedTime();
    
    private DcMotorEx TurretMotor;
    private DcMotor Intake;
    private DcMotorEx Shooter1;
    private DcMotorEx Shooter2;
    
    private Servo LeftServo;
    private Servo RightServo;
    private Servo BackServo;
    
    boolean isAllowedID(int id) {
        for (int allowed : TowerTags) {
            if (allowed == id) return true;
        }
        return false;
    }
    
    boolean Motif1 (int id) {
        for (int allowed : Motif1ID) { if (allowed == id) return true; } return false;
    }
    boolean Motif2 (int id) {
        for (int allowed : Motif2ID) { if (allowed == id) return true; } return false;
    }
    boolean Motif3 (int id) {
        for (int allowed : Motif3ID) { if (allowed == id) return true; } return false;
    }

    @Override
    public void runOpMode() {

       
        MotorL2 = hardwareMap.get(DcMotor.class, "MotorL2");
        MotorR2 = hardwareMap.get(DcMotor.class, "MotorR2");
        MotorL1 = hardwareMap.get(DcMotor.class, "MotorL1");
        MotorR1 = hardwareMap.get(DcMotor.class, "MotorR1");
        
        huskyLens = hardwareMap.get(HuskyLens.class, "camera");
        
        LeftClr = hardwareMap.get(NormalizedColorSensor.class, "LeftClr");
        RightClr = hardwareMap.get(NormalizedColorSensor.class, "RightClr");
        BackClr = hardwareMap.get(NormalizedColorSensor.class, "BackClr");
        
        LeftServo = hardwareMap.get(Servo.class, "LeftServo");
        RightServo = hardwareMap.get(Servo.class, "RightServo");
        BackServo = hardwareMap.get(Servo.class, "BackServo");
        
        TurretMotor  = hardwareMap.get(DcMotorEx.class, "TurretMotor");
        Intake  = hardwareMap.get(DcMotor.class, "Intake");
        Shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        Shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        MotorL2.setDirection(DcMotor.Direction.REVERSE);
        MotorL1.setDirection(DcMotor.Direction.REVERSE);
        
        RightServo.setDirection(Servo.Direction.REVERSE);
        LeftServo.setDirection(Servo.Direction.REVERSE);
        
        Shooter1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Shooter2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        TurretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        Shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        TurretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        TurretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
        double Shoot60Velo = 1375;
        double Shoot75Velo = 2400;
        
        while(!isStarted()) 
        {
            ArtifactsOrder();
            telemetry.addData("Status", "Initialized");
            telemetry.addLine("Press Left Bumper in TeleOp to toggle Turret Auto-Aim!");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            
    
            double speedFactor = 0.3;

            double forward = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x;
  
            double motorL1power = (forward + strafe + rotate) * speedFactor;  
            double motorL2power = (forward - strafe + rotate) * speedFactor;
            double motorR1power = (forward - strafe - rotate) * speedFactor;
            double motorR2power = (forward + strafe - rotate) * speedFactor;
          
            MotorL1.setPower(motorL1power);
            MotorL2.setPower(motorL2power);
            MotorR1.setPower(motorR1power);
            MotorR2.setPower(motorR2power);
            
            
            if (gamepad1.y)
            {
                OrderCheck();
            }
            
            if (gamepad1.a) {
                Shooter1.setVelocity(-Shoot75Velo);
                Shooter2.setVelocity(-Shoot75Velo);    
            } else if (gamepad1.b) {
                Shooter1.setVelocity(-Shoot60Velo);
                Shooter2.setVelocity(-Shoot60Velo);    
            } else {
                Shooter1.setPower(0);
                Shooter2.setPower(0);    
            }
            
            if (gamepad1.dpadDownWasPressed()) shootBack();  
            if (gamepad1.dpadUpWasPressed()) gulgushmek();  
            if (gamepad1.dpadLeftWasPressed()) shootRight();
            if (gamepad1.dpadRightWasPressed()) shootLeft();
            
            if (gamepad1.right_trigger > 0.5) Intake.setPower(1);
            else if (gamepad1.left_trigger > 0.5) Intake.setPower(-1);
            else Intake.setPower(0);
          
            if (gamepad1.left_bumper && !lastLeftBumperState) {
                isTrackingEnabled = !isTrackingEnabled; 
            }
            lastLeftBumperState = gamepad1.left_bumper; 
            

            HuskyLens.Block[] blocks = huskyLens.blocks();
            
            NormalizedRGBA colors1 = LeftClr.getNormalizedColors();
            NormalizedRGBA colors2 = RightClr.getNormalizedColors();
            NormalizedRGBA colors3 = BackClr.getNormalizedColors();
            hue1 = JavaUtil.colorToHue(colors1.toColor());
            hue2 = JavaUtil.colorToHue(colors2.toColor());
            hue3 = JavaUtil.colorToHue(colors3.toColor());
            
           
            if (isTrackingEnabled) 
            {
            
                centerAllowedTag(blocks);
            } 

            updateTelemetry(colors1, colors2, colors3, blocks);
        }
    }
    

    void centerAllowedTag(HuskyLens.Block[] blocks) {
        int centerX = 160; // HuskyLens width is 320, so 160 is the middle
        boolean tagFound = false;

        for (HuskyLens.Block block : blocks) {
            if (isAllowedID(block.id)) {
                tagFound = true;
                
                int error = centerX - block.x;

    
                double maxVelocity = 400; // 
                double gain = 1.5; 

               
                double velocity = Range.clip(error * gain, -maxVelocity, maxVelocity);   

                if (Math.abs(error) < 8) {
                    TurretMotor.setVelocity(0);
                } else {
                    TurretMotor.setVelocity(velocity);
                }
                
                return; // Stop looking at other tags once we find the right one
            }
        }

        
        if (!tagFound) {
            TurretMotor.setVelocity(0);
        }
    }
    
    void updateTelemetry(NormalizedRGBA colors1, NormalizedRGBA colors2, NormalizedRGBA colors3, HuskyLens.Block[] blocks) {
        clrsnsleft();
        clrsnsright();
        clrsnsback();
         
        boolean foundAllowed = false;
    
        if (blocks != null && blocks.length > 0) {
            for (HuskyLens.Block block : blocks) {
                if (isAllowedID(block.id)) {
                    foundAllowed = true;
                    telemetry.addData("Target Tag ID", block.id);
                    telemetry.addData("Error (Pixels)", 160 - block.x);
                }
            }
        }
        
        if (!foundAllowed) {
            telemetry.addLine("NO TARGET FOUND");
        }
        
        
        if (isTrackingEnabled) {
            telemetry.addLine(">> AUTO-AIM IS: [ON]");
        } else {
            telemetry.addLine(">> AUTO-AIM IS: [OFF]");
        }
        
        
        telemetry.addData("Balls Order:", order);
        telemetry.update();
    }

    void OrderCheck() {
        if (order == 1) Order1();
        else if (order == 2) { Order2(); }
        else if (order == 3) { Order3(); }
    }
   
    void Order1(){
        if ((hue1>=122 && hue1<=180) && (hue2>=200 && hue2<=260) && (hue3>=200 && hue3<=260)) shootLeftRightBack();
        else if ((hue1>=200 && hue1<=260) && (hue2>=122 && hue2<=180) && (hue3>=200 && hue3<=260)) shootRightLeftBack();
        else if ((hue1>=200 && hue1<=260) && (hue2>=200 && hue2<=260) && (hue3>=122 && hue3<=180)) shootBackLeftRight();
        else if ((hue1>=200 && hue1<=260) && (hue2>=200 && hue2<=260) && (hue3>=200 && hue3<=260)) shootLeftRightBack();
        else if ((hue1>=122 && hue1<=180) && (hue2>=122 && hue2<=180) && (hue3>=122 && hue3<=180)) shootLeftRightBack();
        else if ((hue1>=122 && hue1<=180 || hue1>=200 && hue1<=260) && (hue2>=122 && hue2<=180 || hue2>=200 && hue2<=260) && !(hue3>=122 && hue3<=180 || hue3>=200 && hue3<=260)) shootLeftRight();
        else if ((hue1>=122 && hue1<=180 || hue1>=200 && hue1<=260) && (hue3>=122 && hue3<=180 || hue3>=200 && hue3<=260) && !(hue2>=122 && hue2<=180 || hue2>=200 && hue2<=260)) shootLeftBack();
        else if ((hue2>=122 && hue2<=180 || hue2>=200 && hue2<=260) && (hue3>=122 && hue3<=180 || hue3>=200 && hue3<=260) && !(hue1>=122 && hue1<=180 || hue1>=200 && hue1<=260)) shootRightBack();
    }
  
    void Order2(){
        if ((hue1>=122 && hue1<=180) && (hue2>=200 && hue2<=260) && (hue3>=200 && hue3<=260)) shootRightLeftBack(); 
        else if ((hue1>=200 && hue1<=260) && (hue2>=122 && hue2<=180) && (hue3>=200 && hue3<=260)) shootLeftRightBack(); 
        else if ((hue1>=200 && hue1<=260) && (hue2>=200 && hue2<=260) && (hue3>=122 && hue3<=180)) shootLeftBackRight(); 
        else if ((hue1>=200 && hue1<=260) && (hue2>=200 && hue2<=260) && (hue3>=200 && hue3<=260)) shootLeftRightBack();
        else if ((hue1>=122 && hue1<=180) && (hue2>=122 && hue2<=180) && (hue3>=122 && hue3<=180)) shootLeftRightBack();
        else if ((hue1>=122 && hue1<=180 || hue1>=200 && hue1<=260) && (hue2>=122 && hue2<=180 || hue2>=200 && hue2<=260) && !(hue3>=122 && hue3<=180 || hue3>=200 && hue3<=260)) shootLeftRight();
        else if ((hue1>=122 && hue1<=180 || hue1>=200 && hue1<=260) && (hue3>=122 && hue3<=180 || hue3>=200 && hue3<=260) && !(hue2>=122 && hue2<=180 || hue2>=200 && hue2<=260)) shootLeftBack();
        else if ((hue2>=122 && hue2<=180 || hue2>=200 && hue2<=260) && (hue3>=122 && hue3<=180 || hue3>=200 && hue3<=260) && !(hue1>=122 && hue1<=180 || hue1>=200 && hue1<=260)) shootRightBack();
    }
  
    void Order3(){
        if ((hue1>=122 && hue1<=180) && (hue2>=200 && hue2<=260) && (hue3>=200 && hue3<=260)) shootRightBackLeft();
        else if ((hue1>=200 && hue1<=260) && (hue2>=122 && hue2<=180) && (hue3>=200 && hue3<=260)) shootLeftBackRight();
        else if ((hue1>=200 && hue1<=260) && (hue2>=200 && hue2<=260) && (hue3>=122 && hue3<=180)) shootLeftRightBack();
        else if ((hue1>=200 && hue1<=260) && (hue2>=200 && hue2<=260) && (hue3>=200 && hue3<=260)) shootLeftRightBack();
        else if ((hue1>=122 && hue1<=180) && (hue2>=122 && hue2<=180) && (hue3>=122 && hue3<=180)) shootLeftRightBack();
        else if ((hue1>=122 && hue1<=180 || hue1>=200 && hue1<=260) && (hue2>=122 && hue2<=180 || hue2>=200 && hue2<=260) && !(hue3>=122 && hue3<=180 || hue3>=200 && hue3<=260)) shootLeftRight();
        else if ((hue1>=122 && hue1<=180 || hue1>=200 && hue1<=260) && (hue3>=122 && hue3<=180 || hue3>=200 && hue3<=260) && !(hue2>=122 && hue2<=180 || hue2>=200 && hue2<=260)) shootLeftBack();
        else if ((hue2>=122 && hue2<=180 || hue2>=200 && hue2<=260) && (hue3>=122 && hue3<=180 || hue3>=200 && hue3<=260) && !(hue1>=122 && hue1<=180 || hue1>=200 && hue1<=260)) shootRightBack();
    }

    void shootLeftRightBack() { shootLeft(); sleep(100); shootRight(); sleep(100); shootBack(); }
    void shootRightLeftBack() { shootRight(); sleep(100); shootLeft(); sleep(100); shootBack(); }
    void shootBackLeftRight() { shootBack(); sleep(100); shootLeft(); sleep(100); shootRight(); }
    void shootRightBackLeft() { shootRight(); shootBack(); shootLeft(); }
    void shootLeftBackRight() { shootLeft(); shootBack(); shootRight(); }

    void shootLeftRight() { shootLeft(); sleep(100); shootRight(); }
    void shootLeftBack() { shootLeft(); sleep(100); shootBack(); }
    void shootRightBack() { shootRight(); sleep(100); shootBack(); }

    void shootLeft(){
        LeftServo.setPosition(0.5);
        sleep(700);
        LeftServo.setPosition(0);
        sleep(700);
    }
    void shootRight(){
        RightServo.setPosition(0);
        sleep(700);
        RightServo.setPosition(0.85);
        sleep(700);
    }
    void shootBack(){
        BackServo.setPosition(0.65);
        sleep(700);
        BackServo.setPosition(1);
        sleep(700);
    }

    void clrsnsleft(){  }
    void clrsnsright(){  }
    void clrsnsback(){  }

    void ArtifactsOrder (){
        HuskyLens.Block[] blocks = huskyLens.blocks();
        for (HuskyLens.Block block : blocks) {
            if (Motif1(block.id)) order = 1;
            else if(Motif2(block.id)) order = 2;
            else if(Motif3(block.id)) order = 3;    
        }
    }
    
    void gulgushmek(){
        LeftServo.setPosition(0.1);
        RightServo.setPosition(0.6);
        BackServo.setPosition(0.9);
        sleep(100);
        LeftServo.setPosition(0);
        RightServo.setPosition(0.7);
        BackServo.setPosition(1);
    }
}
