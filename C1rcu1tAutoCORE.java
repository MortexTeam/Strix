package org.firstinspires.ftc.teamcode;

import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

@Autonomous(name="C1rcu1t Auto", group="Scripts")

public class C1rcu1tAuto extends LinearOpMode {

    private final int READ_PERIOD = 1;
    
    int[] TowerTags = {4,5};
    int[] Motif1ID = {1};
    int[] Motif2ID = {2};
    int[] Motif3ID = {3};
    
    double SPEED_GAIN  = 0.03;
    double STRAFE_GAIN = 0.15;
    double TURN_GAIN   = 0.03;
    
    double hue1;
    double hue2;
    double hue3;
    
    int PosLeft1 = 0.5 ;
    int PosRight1 = 0 ;
    int PosBack1 = 0.65 ;
    
    int PosLeft2 = 0;
    int PosRight2 = 0.7;
    int PosBack2 = 1;
    
    int order = 0;
    
    private HuskyLens huskyLens;
    
    private NormalizedColorSensor LeftClr;
    private NormalizedColorSensor RightClr;
    private NormalizedColorSensor BackClr;

    final double MAX_AUTO_SPEED = 0.2;
    final double MAX_AUTO_STRAFE= 0.2;
    final double MAX_AUTO_TURN  = 0.2;

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    
    private DcMotor TurretMotor;
    private DcMotor Intake;
    private DcMotor Shooter1;
    private DcMotor Shooter2;
    
    private Servo LeftServo;
    private Servo RightServo;
    private Servo BackServo;

    private SparkFunOTOS myOtos;
    SparkFunOTOS.Pose2D pos;

    boolean isAllowedID(int id) {
        for (int allowed : TowerTags) {
            if (allowed == id) {
                return true;
            }
        }
        return false;
    }//Tower AprilTasgs Recognition
    boolean Motif1 (int id) {
        for (int allowed : Motif1ID) {
            if (allowed == id) {
                return true;
            }
        }
        return false;
    }//Motif1 AprilTags Recognition
    boolean Motif2 (int id) {
        for (int allowed : Motif2ID) {
            if (allowed == id) {
                return true;
            }
        }
        return false;
    }//Motif2 AprilTags Recognition
    boolean Motif3 (int id) {
        for (int allowed : Motif3ID) {
            if (allowed == id) {
                return true;
            }
        }
        return false;
    }//Motif3 AprilTags Recognition

    @Override
    public void runOpMode() {
        
        huskyLens = hardwareMap.get(HuskyLens.class, "camera");//Camera 
        
        LeftClr = hardwareMap.get(NormalizedColorSensor.class, "LeftClr");//   }
        RightClr = hardwareMap.get(NormalizedColorSensor.class, "RightClr");// |-Color sensors
        BackClr = hardwareMap.get(NormalizedColorSensor.class, "BackClr");//   }
        
        LeftServo = hardwareMap.get(Servo.class, "LeftServo");//   }
        RightServo = hardwareMap.get(Servo.class, "RightServo");// |-Servos
        BackServo = hardwareMap.get(Servo.class, "BackServo");//   }

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "MotorL1");// }
        leftBackDrive   = hardwareMap.get(DcMotor.class, "MotorL2");// |
        rightFrontDrive = hardwareMap.get(DcMotor.class, "MotorR1");// |-Motors
        rightBackDrive  = hardwareMap.get(DcMotor.class, "MotorR2");// }
        
        TurretMotor  = hardwareMap.get(DcMotor.class, "TurretMotor");// }
        Intake  = hardwareMap.get(DcMotor.class, "Intake");//           |
        Shooter1 = hardwareMap.get(DcMotor.class, "Shooter1");//        |-Turret & Shooters motor
        Shooter2 = hardwareMap.get(DcMotor.class, "Shooter2");//        }        
        
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);// }
        TurretMotor.setDirection(DcMotor.Direction.REVERSE);//     }-Motor REVERSE
        
        RightServo.setDirection(Servo.Direction.REVERSE);// }
        LeftServo.setDirection(Servo.Direction.REVERSE);//  }-Servo REVERSE

        myOtos = hardwareMap.get(SparkFunOTOS.class, "otos");//Sparkfun OTOS

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        configureOtos();

        while(!isStarted()) {
           
            
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }// Init scan AprilTag

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.update();
        waitForStart();
        
       
        while(opModeIsActive()) {
            NormalizedRGBA colors1 = LeftClr.getNormalizedColors();
            NormalizedRGBA colors2 = RightClr.getNormalizedColors();
            NormalizedRGBA colors3 = BackClr.getNormalizedColors();
            
            hue1 = JavaUtil.colorToHue(colors1.toColor());
            hue2 = JavaUtil.colorToHue(colors2.toColor());
            hue3 = JavaUtil.colorToHue(colors3.toColor());
            
            

            HuskyLens.Block[] blocks = huskyLens.blocks();

            updateTelemetry(colors1, colors2, colors3, blocks);

            if (!rateLimit.hasExpired()) 
            {
                continue;
            }

            rateLimit.reset();
            
            
            otosDrive(0, 0, 0, 0);// otosDrive (X coordinate, Y coordinate, Robots Angle, Timer to complete command)
            
            ArtifactsOrder();
            
            centerAllowedTag();
            
            OrderCheck();

            telemetry.update();
        }

        telemetry.addData("Status", "Running");
        telemetry.update();
        centerAllowedTag();
    }


  void updateTelemetry(NormalizedRGBA colors1, NormalizedRGBA colors2, NormalizedRGBA colors3, HuskyLens.Block[] blocks) {
    //Color sensors function
     clrsnsleft();
     clrsnsright();
     clrsnsback();
     
    //Camera tagies
    boolean foundAllowed = false;

    if (blocks != null && blocks.length > 0) 
    {
        for (HuskyLens.Block block : blocks) 
        {

            if (isAllowedID(block.id)) 
            {
                foundAllowed = true;
                telemetry.addData("Target Tag ID", block.id);
                telemetry.addData("Target X", block.x);
                telemetry.addData("Error", 160 - block.x);
            }
        }
    }
    

    // ----- OTOS -----
    SparkFunOTOS.Pose2D currentPos = myPosition();
    telemetry.addData("OTOS X", currentPos.x);
    telemetry.addData("OTOS Y", currentPos.y);
    telemetry.addData("OTOS Heading", currentPos.h);
    telemetry.addData("Balls Order:", order);
    telemetry.addData("Hue1:", hue1);
    telemetry.addData("Hue2:", hue2);
    telemetry.addData("Hue3:", hue3);
    telemetry.update();
}//Telemetry Function

  private void configureOtos(){

        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        myOtos.setLinearUnit(DistanceUnit.CM);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        myOtos.setLinearScalar(1.008);
        myOtos.setAngularScalar(0.992);

        myOtos.calibrateImu();
        myOtos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }//Configuring Sparkfun OTOS function

  void centerAllowedTag(){

        DcMotor turretMotor = TurretMotor;

        HuskyLens.Block[] blocks = huskyLens.blocks();

        int centerX = 160;

        for (HuskyLens.Block block : blocks) 
        {

            if (isAllowedID(block.id)) {

                int error = centerX - block.x;

                double gain = 0.01;
                double power = Range.clip(error * gain, -0.3, 0.3);

                if (Math.abs(error) < 8) {
                    turretMotor.setPower(0);
                } else {
                    turretMotor.setPower(power);
                }
                return;
            }
        }

        turretMotor.setPower(0);
    }//Centering robots Turret towards right Tower
    
   void TurretRange(){

    DcMotorEx shotah2 = Shooter1;
    DcMotorEx shotah1 = Shooter2;

    HuskyLens.Block[] blocks = huskyLens.blocks();

    for (HuskyLens.Block block : blocks)
    {

        if (isAllowedID(block.id)) {

            int size = block.width * block.height;   // tag size (area)

            // smaller tag = bigger speed
            double baseVelocity = 2000;              // max speed
            double minVelocity = 700;                // minimum speed

            double velocity = baseVelocity / (1 + size * 0.01);
            velocity = Range.clip(velocity, minVelocity, baseVelocity);

            shotah1.setVelocity(velocity);
            shotah2.setVelocity(velocity);

            return;
        }
    }

    shotah1.setVelocity(0);
    shotah2.setVelocity(0);
}//Centering robots Turret towards right Tower

  void otosDrive(double targetX, double targetY, double targetHeading, int maxTime) {

        double drive, strafe, turn;
        double xError, yError, yawError;

        SparkFunOTOS.Pose2D currentPos = myPosition();
        xError = targetX-currentPos.x;
        yError = targetY-currentPos.y;
        yawError = targetHeading-currentPos.h;

        runtime.reset();

        while(opModeIsActive() && (runtime.milliseconds() < maxTime*1000) &&
        ((Math.abs(xError) > 1.5) || (Math.abs(yError) > 1.5) || (Math.abs(yawError) > 4))) 
        {

            drive  = Range.clip(xError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            strafe = Range.clip(yError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            turn   = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            telemetry.update();

            moveRobot(strafe, drive, turn);

            currentPos = myPosition();
            xError = targetX-currentPos.x;
            yError = targetY-currentPos.y;
            yawError = targetHeading-currentPos.h;
        }

        moveRobot(0,0,0);
    }//Sparkfun OTOS robot moving function

  SparkFunOTOS.Pose2D myPosition() {
        pos = myOtos.getPosition();
        return new SparkFunOTOS.Pose2D(pos.y, pos.x, -pos.h);
    }//Creating position parametr

  void moveRobot(double x, double y, double yaw) {

        double leftFrontPower  = x + y + yaw;
        double rightFrontPower = x - y - yaw;
        double leftBackPower   = x - y + yaw;
        double rightBackPower  = x + y - yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        sleep(10);
    }//Robot Moving function
    
  void ArtifactsOrder (){
        HuskyLens.Block[] blocks = huskyLens.blocks();
        
        for (HuskyLens.Block block : blocks) 
        {
            if (Motif1(block.id)) 
            {
              order = 1;
            }
            else if(Motif2(block.id))
            {
              order = 2;
            }
            else if(Motif3(block.id))
            {
              order = 3;    
            }
        }
    }//Scanning AprilTags from Obelisk
    
  void clrsnsleft(){
     if(hue1 < 30){
        telemetry.addData("Color", "Red");
    }
    else if (hue1 < 60) {
        telemetry.addData("Color", "Orange");
    }
    else if (hue1 < 90){
        telemetry.addData("Color", "Yellow");
    }
    else if (hue1 < 150){
        telemetry.addData("Color", "Green");
    }
    else if (hue1 < 225){
        telemetry.addData("Color", "Blue");
    }
    else if (hue1 < 350){
        telemetry.addData("Color", "Purple");
    }
    else{
        telemetry.addData("Color", "Red");
    }   
   } //Left sensor telemetry
    
  void clrsnsright(){
     if(hue2 < 30){
        telemetry.addData("Color", "Red");
    }
    else if (hue2 < 60) {
        telemetry.addData("Color", "Orange");
    }
    else if (hue2 < 90){
        telemetry.addData("Color", "Yellow");
    }
    else if (hue2 < 150){
        telemetry.addData("Color", "Green");
    }
    else if (hue2 < 225){
        telemetry.addData("Color", "Blue");
    }
    else if (hue2 < 350){
        telemetry.addData("Color", "Purple");
    }
    else{
        telemetry.addData("Color", "Red");
    }   
   }//Right sensor telemetry
    
  void clrsnsback(){
     if(hue3 < 30){
        telemetry.addData("Color", "Red");
    }
    else if (hue3 < 60) {
        telemetry.addData("Color", "Orange");
    }
    else if (hue3 < 90){
        telemetry.addData("Color", "Yellow");
    }
    else if (hue3 < 150){
        telemetry.addData("Color", "Green");
    }
    else if (hue3 < 225){
        telemetry.addData("Color", "Blue");
    }
    else if (hue3 < 350){
        telemetry.addData("Color", "Purple");
    }
    else{
        telemetry.addData("Color", "Red");
    }
   }//Back sensor telemetry
   
  void OrderCheck(){
        if (order == 1) Order1();
        else if (order == 2) { Order2(); }
        else if (order == 3) { Order3(); }
    }//Changing "order" int depending on AprilTag ID
   
  void Order1(){

  
    if ((hue1>=122 && hue1<=180) && (hue2>=200 && hue2<=260) && (hue3>=200 && hue3<=260))
    {  
        shootLeftRightBack();
    }

    else if ((hue1>=200 && hue1<=260) && (hue2>=122 && hue2<=180) && (hue3>=200 && hue3<=260))
    {  
        shootRightLeftBack();
    }

    else if ((hue1>=200 && hue1<=260) && (hue2>=200 && hue2<=260) && (hue3>=122 && hue3<=180))
    {  
        shootBackLeftRight();
    }
    
    else if ((hue1>=200 && hue1<=260) && (hue2>=200 && hue2<=260) && (hue3>=200 && hue3<=260))
    {
        shootLeftRightBack();
    }
    
    else if ((hue1>=122 && hue1<=180) && (hue2>=122 && hue2<=180) && (hue3>=122 && hue3<=180))
    {
        shootLeftRightBack();
    }

    else if ((hue1>=122 && hue1<=180 || hue1>=200 && hue1<=260) &&
             (hue2>=122 && hue2<=180 || hue2>=200 && hue2<=260) &&
             !(hue3>=122 && hue3<=180 || hue3>=200 && hue3<=260))
    {
        shootLeftRight();
    }
    
    else if ((hue1>=122 && hue1<=180 || hue1>=200 && hue1<=260) &&
             (hue3>=122 && hue3<=180 || hue3>=200 && hue3<=260) &&
             !(hue2>=122 && hue2<=180 || hue2>=200 && hue2<=260))
    {
        shootLeftBack();
    }

    else if ((hue2>=122 && hue2<=180 || hue2>=200 && hue2<=260) &&
             (hue3>=122 && hue3<=180 || hue3>=200 && hue3<=260) &&
             !(hue1>=122 && hue1<=180 || hue1>=200 && hue1<=260))
    {
        shootRightBack();
    }
  }//First Order Function
  
  void Order2(){
    
    if ((hue1>=122 && hue1<=180) && (hue2>=200 && hue2<=260) && (hue3>=200 && hue3<=260))
    {
        shootRightLeftBack(); // purple, green, purple
    }

    else if ((hue1>=200 && hue1<=260) && (hue2>=122 && hue2<=180) && (hue3>=200 && hue3<=260))
    {
        shootLeftRightBack(); 
    }

    else if ((hue1>=200 && hue1<=260) && (hue2>=200 && hue2<=260) && (hue3>=122 && hue3<=180))
    {
        shootLeftBackRight(); 
    }
    
    else if ((hue1>=200 && hue1<=260) && (hue2>=200 && hue2<=260) && (hue3>=200 && hue3<=260))
    {
        shootLeftRightBack();
    }
    
    else if ((hue1>=122 && hue1<=180) && (hue2>=122 && hue2<=180) && (hue3>=122 && hue3<=180))
    {
        shootLeftRightBack();
    }

    else if ((hue1>=122 && hue1<=180 || hue1>=200 && hue1<=260) &&
             (hue2>=122 && hue2<=180 || hue2>=200 && hue2<=260) &&
             !(hue3>=122 && hue3<=180 || hue3>=200 && hue3<=260))
    {
        shootLeftRight();
    }

    else if ((hue1>=122 && hue1<=180 || hue1>=200 && hue1<=260) &&
             (hue3>=122 && hue3<=180 || hue3>=200 && hue3<=260) &&
             !(hue2>=122 && hue2<=180 || hue2>=200 && hue2<=260))
    {
        shootLeftBack();
    }
    
    else if ((hue2>=122 && hue2<=180 || hue2>=200 && hue2<=260) &&
             (hue3>=122 && hue3<=180 || hue3>=200 && hue3<=260) &&
             !(hue1>=122 && hue1<=180 || hue1>=200 && hue1<=260))
    {
        shootRightBack();
    }
}//Second Order Function
  
  void Order3(){

    
    if ((hue1>=122 && hue1<=180) && (hue2>=200 && hue2<=260) && (hue3>=200 && hue3<=260))
    {   
        shootRightBackLeft();
    }

    else if ((hue1>=200 && hue1<=260) && (hue2>=122 && hue2<=180) && (hue3>=200 && hue3<=260))
    {   
        shootLeftBackRight();
    }

    else if ((hue1>=200 && hue1<=260) && (hue2>=200 && hue2<=260) && (hue3>=122 && hue3<=180))
    {  
        shootLeftRightBack();
    }
    
    else if ((hue1>=200 && hue1<=260) && (hue2>=200 && hue2<=260) && (hue3>=200 && hue3<=260))
    {
        shootLeftRightBack();
    }
    
    else if ((hue1>=122 && hue1<=180) && (hue2>=122 && hue2<=180) && (hue3>=122 && hue3<=180))
    {
        shootLeftRightBack();
    }

    else if ((hue1>=122 && hue1<=180 || hue1>=200 && hue1<=260) &&
             (hue2>=122 && hue2<=180 || hue2>=200 && hue2<=260) &&
             !(hue3>=122 && hue3<=180 || hue3>=200 && hue3<=260))
    {
        shootLeftRight();
    }

    else if ((hue1>=122 && hue1<=180 || hue1>=200 && hue1<=260) &&
             (hue3>=122 && hue3<=180 || hue3>=200 && hue3<=260) &&
             !(hue2>=122 && hue2<=180 || hue2>=200 && hue2<=260))
    {
        shootLeftBack();
    }
    
    else if ((hue2>=122 && hue2<=180 || hue2>=200 && hue2<=260) &&
             (hue3>=122 && hue3<=180 || hue3>=200 && hue3<=260) &&
             !(hue1>=122 && hue1<=180 || hue1>=200 && hue1<=260))
    {
        shootRightBack();
    }
}//Third Order Function

  void shootLeftRightBack() {
    shootLeft();
    sleep(100);
    shootRight();
    sleep(100);
    shootBack();
}//All servos shooting 1
  void shootRightLeftBack() {
    shootRight();
    sleep(100);
    shootLeft();
    sleep(100);
    shootBack();
}//All servos shooting 2
  void shootBackLeftRight() {
    shootBack();
    sleep(100);
    shootLeft();
    sleep(100);
    shootRight();
}//All servos shooting 3
  void shootRightBackLeft() {
    shootRight();
    shootBack();
    shootLeft();
}//All servos shooting 4
  void shootLeftBackRight() {
    shootLeft();
    shootBack();
    shootRight();
}//All servos shooting 5

  void shootLeftRight() {
    shootLeft();
    sleep(100);
    shootRight();
}// L/R servos shooting
  void shootLeftBack() {
    shootLeft();
    sleep(100);
    shootBack();
}// L/B servos shooting
  void shootRightBack() {
    shootRight();
    sleep(100);
    shootBack();
}// R/B servos shooting 

  void shootLeft() {
    LeftServo.setPosition(PosLeft1);
    sleep(700);
    LeftServo.setPosition(PosLeft2);
    sleep(700);
}//L servo shooting
  void shootRight() {
    RightServo.setPosition(PosRight1);
    sleep(700);
    RightServo.setPosition(PosRight2);
    sleep(700);
}//R servo shooting
  void shootBack() {
    BackServo.setPosition(PosBack1);
    sleep(700);
    BackServo.setPosition(PosBack2);
    sleep(700);
 }//B servo shooting
  
}
