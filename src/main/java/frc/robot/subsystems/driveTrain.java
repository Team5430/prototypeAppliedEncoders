package frc.robot.subsystems;


import org.checkerframework.checker.units.qual.min;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
import java.lang.Math;

import java.util.Map;

public class driveTrain extends SubsystemBase  {  

   final static  WPI_TalonFX backRightMotor = new WPI_TalonFX(Constants.CANid.kBackRightFX);
   final static  WPI_TalonFX backLeftMotor = new WPI_TalonFX(Constants.CANid.kBackLeftFX);
   final static  WPI_TalonFX frontRightMotor = new WPI_TalonFX(Constants.CANid.kFrontRightFX);
   final static  WPI_TalonFX frontLeftMotor = new WPI_TalonFX(Constants.CANid.kFrontLeftFX);
      //organizes motor conrollers into groups, left and right respectively
    final static MotorControllerGroup leftGroup = new MotorControllerGroup(backLeftMotor, frontLeftMotor);
    final static MotorControllerGroup rightGroup = new MotorControllerGroup(backRightMotor, frontRightMotor);
    static SupplyCurrentLimitConfiguration configTalonCurrent = new SupplyCurrentLimitConfiguration(true,55,0,0);


    //NetworkTableEntry distanceEntry = SmartDashboard.getEntry("Dist (in)");
    
    //gyro
    final static AHRS ahrs = new AHRS(Port.kUSB1);

    static Timer dTimer = new Timer();    

//Motor settings
    public static void driveSettings(){
      frontRightMotor.configSupplyCurrentLimit(configTalonCurrent);
      frontLeftMotor.configSupplyCurrentLimit(configTalonCurrent);
      backLeftMotor.configSupplyCurrentLimit(configTalonCurrent);
      backRightMotor.configSupplyCurrentLimit(configTalonCurrent);

      //This one will attempt to create slider widgets for power and distance.
      initShuffleBoardWidgets();
      //This one will attempt to create just the number cells for power and distance.
      //initShuffleVals();
      
      
    }

//drive with input  
    public void Drive(double left, double right){
    //sin used to smooth out the rate of acceleration
    if(Constants.multiplier < 0){
      
      leftGroup.set(Math.sin(left * Constants.multiplier)* -Math.sin(left * Constants.multiplier));
      rightGroup.set(Math.sin(-right * Constants.multiplier) * -Math.sin(-right * Constants.multiplier)); 
    }else{
      leftGroup.set(Math.sin(left * Constants.multiplier)* Math.sin(left * Constants.multiplier));
      rightGroup.set(Math.sin(-right * Constants.multiplier) * Math.sin(-right * Constants.multiplier));
    }
  }
//Auto Drive

    public static void AutoDrive(double left, double right){

      leftGroup.set(left);
      rightGroup.set(-right);
    }
  //VariableSpeed
    public void VariableSpeedIncrease(){
      Constants.multiplier += .1;
    }
    public void VariableSpeedDecrease(){
      Constants.multiplier -= .1;
    }
    
  //Drive in Time
    public void driveinTime(double time, double power){
      dTimer.start();
         while(dTimer.get() <= time){
             AutoDrive(power, power);}
        AutoDrive(0, 0);
     dTimer.reset();
          dTimer.stop();
    }
    //Drive in distance
    public void driveInDistance(double distanceinInches, double power){
        Constants.previousEncoderPos = Constants.encoderPos;
      while ((( Constants.encoderPos - Constants.previousEncoderPos) * Constants.wheelCircumference) / 12 < distanceinInches){
        AutoDrive(power, -power);
        }
      }

    //Drive in distance
    public void driveInDistanceUpdated(){
      //JL, values will attempt to be read from Shuffleboard
      double distanceinInches = SmartDashboard.getNumber("Dist (in)", 60);
      double power = SmartDashboard.getNumber("Power", 0.25);
      //JL, sets power to negative if approaching negative distance
      if (distanceinInches < 0){
      power *= -1;
    }
      //JL, previous rots holds the position prior to movement for relative distance
        Constants.previousLeftWheelRots = Constants.leftWheelRots;
        Constants.previousRightWheelRots = Constants.rightWheelRots;
      while ((Math.abs((( Constants.leftWheelRots - Constants.previousLeftWheelRots) * Constants.wheelCircumference)) < Math.abs(distanceinInches)) || 
      ((( Math.abs(Constants.rightWheelRots - Constants.previousRightWheelRots) * Constants.wheelCircumference)) < Math.abs(distanceinInches))){
        
          if(Math.abs(((Constants.leftWheelRots - Constants.previousLeftWheelRots) * Constants.wheelCircumference)) < Math.abs(distanceinInches)){
            leftGroup.set(power);
          }
          else{
            leftGroup.set(0);
          }
          if(Math.abs((( Constants.rightWheelRots - Constants.previousRightWheelRots) * Constants.wheelCircumference)) < Math.abs(distanceinInches)){
            rightGroup.set(power);
          }else{
            rightGroup.set(0);
            
          }
        }
      }

      

      public void easyWayOut(){
        double distanceinInches = 60;
        double power = 0.25;
          Constants.previousLeftWheelRots = Constants.leftWheelRots;
          Constants.previousRightWheelRots = Constants.rightWheelRots;
        while (((( Constants.leftWheelRots - Constants.previousLeftWheelRots) * Constants.wheelCircumference) < distanceinInches) || 
        ((( Constants.rightWheelRots - Constants.previousRightWheelRots) * Constants.wheelCircumference) < distanceinInches)){
          
            if((( Constants.leftWheelRots - Constants.previousLeftWheelRots) * Constants.wheelCircumference) < distanceinInches){
              leftGroup.set(power);
            }
            else{
              leftGroup.set(0);
            }
            if((( Constants.rightWheelRots - Constants.previousRightWheelRots) * Constants.wheelCircumference) < distanceinInches){
              rightGroup.set(power);
            }else{
              rightGroup.set(0);
            }
          }
        }


      //going to assume a wheel on the turning point spins once every two rotation on the turning 1:2
      //WIP needs to have another variable that determines how long they turn! 
      public void turn90degrees(String direction){
        switch(direction){
          case "left":
        leftGroup.set(0.3);
        rightGroup.set(-0.15);

          case "right":
          leftGroup.set(-0.15);
          rightGroup.set(0.3);
        }
      }
   
      

    public void calibrateGyro(){  
      // Calibrates the Gyro at beginning of lifetime. DO NOT TOUCH IT WHILE IT IS CALIBRATING!
      ahrs.calibrate();
    }
    public void gyro0Yaw(){
    //Sets Yaw (aka the z-axis)
       ahrs.zeroYaw();
   }  
   
     public static double getGyroAngle(){//acquire the angle for the Gyro
      return ahrs.getAngle();
    }
    
    public static double getGyroPitch(){//get the pitch for the Gyro
      return ahrs.getPitch();
    }

    public static double getGyroYaw(){//Get the value for the Yah in the Gyro
      return ahrs.getYaw();
    }

    public static double getGyroroll(){//
      return ahrs.getRoll();
    }

    public static double getAccelX(){
      return ahrs.getWorldLinearAccelX();
    }

    public static double getAccelY(){
      return ahrs.getWorldLinearAccelY();
    }

    public static double getAccelz(){
      return ahrs.getWorldLinearAccelZ();
    }
  //commands 


  

  public static void initShuffleBoardWidgets(){
    Shuffleboard.getTab("SmartDashboard")
      .add("Dist (in)", 60)
      .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
      .withProperties(Map.of("min", -100, "max", 100))
      .getEntry();
    Shuffleboard.getTab("SmartDashboard")
      .add("Power", 0.25)
      .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
      .withProperties(Map.of("min", -1, "max", 1))
      .getEntry();
    //Shuffleboard.getTab("Smartdashboard").
  }

  public static void initShuffleVals(){

    SmartDashboard.putNumber("Dist (in)", 60);
    SmartDashboard.putNumber("Power", 0.25);

  }

    @Override
    public void periodic() {

     Constants.encoderPos = (backLeftMotor.getSelectedSensorPosition() / 2048) * 360;
     //every revolution on the motor is now worth 360, 
      double encoderVel = (backLeftMotor.getSelectedSensorVelocity() / 2048) * 360 * 10;
      Constants.error = Constants.wanted - Constants.encoderPos;

      Constants.leftEncoderRots = (backLeftMotor.getSelectedSensorPosition() / 2048);
      Constants.rightEncoderRots = (backRightMotor.getSelectedSensorPosition() / 2048) ;

      Constants.leftWheelRots = Constants.leftEncoderRots / 10.71;
      Constants.rightWheelRots = Constants.rightEncoderRots / 10.71;
      
      //every revolution on the motor is now worth 360, 
    


  // JL: This deploys data to the SmartDashboard directly, but can't do much more than that.
      SmartDashboard.putNumber("Multiplier", Constants.multiplier); 
      SmartDashboard.putNumber("Velocity", encoderVel);
      SmartDashboard.putNumber("Degrees", encoderVel);
      SmartDashboard.putNumber("Angle", driveTrain.getGyroAngle());
      SmartDashboard.putNumber("Z Axis", driveTrain.getGyroYaw());
      SmartDashboard.putNumber("X Accel", driveTrain.getAccelX());
      SmartDashboard.putNumber("Y Accel", driveTrain.getAccelY());
      SmartDashboard.putNumber("Z Accel", driveTrain.getAccelz());
      SmartDashboard.putNumber("Pitch",  driveTrain.getGyroPitch());

      SmartDashboard.putNumber("Total Left Dist Travelled", Constants.leftWheelRots * Constants.wheelCircumference);
      SmartDashboard.putNumber("Total Right Dist Travelled", Constants.rightWheelRots * Constants.wheelCircumference);

      SmartDashboard.putNumber("Total Left Encoder Rotations", Constants.leftEncoderRots);
      SmartDashboard.putNumber("Total Right Encoder Rotations", Constants.rightEncoderRots);
      
/*     
//Widget testing on shuffleboard 
    Shuffleboard.getTab("SmartDashboard")
      .add("Angle", driveTrain.getGyroAngle()).withWidget(BuiltInWidgets.kGyro);

    Shuffleboard.getTab("SmartDashboard")
      .add("Yaw", driveTrain.getGyroYaw()).withWidget(BuiltInWidgets.kGyro);

    Shuffleboard.getTab("SmartDashboard")
      .add("Pitch", driveTrain.getGyroPitch()).withWidget(BuiltInWidgets.kGyro);

    Shuffleboard.getTab("SmartDashboard") 
      .add();
    
    Shuffleboard.getTab("SmartDashboard")
      .add("Roll", driveTrain.getGyroroll()).withWidget(BuiltInWidgets.kField);
    
*/

    }
    
  //if you want to add anything, make other functions to use                                                                                                                                  

}