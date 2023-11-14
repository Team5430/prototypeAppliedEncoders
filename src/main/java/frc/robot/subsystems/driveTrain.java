package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
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

public class driveTrain extends SubsystemBase  {

   final static  WPI_TalonFX backRightMotor = new WPI_TalonFX(Constants.CANid.kBackRightFX);
   final static  WPI_TalonFX backLeftMotor = new WPI_TalonFX(Constants.CANid.kBackLeftFX);
   final static  WPI_TalonFX frontRightMotor = new WPI_TalonFX(Constants.CANid.kFrontRightFX);
   final static  WPI_TalonFX frontLeftMotor = new WPI_TalonFX(Constants.CANid.kFrontLeftFX);
   final static WPI_TalonFX testMotor = new WPI_TalonFX(Constants.CANid.kTestMotorFX);


      //organizes motor conrollers into groups, left and right respectively
    final static MotorControllerGroup leftGroup = new MotorControllerGroup(backLeftMotor, frontLeftMotor);
    final static MotorControllerGroup rightGroup = new MotorControllerGroup(backRightMotor, frontRightMotor);
    static SupplyCurrentLimitConfiguration configTalonCurrent = new SupplyCurrentLimitConfiguration(true,55,0,0);

    
    //gyro
    final static AHRS ahrs = new AHRS(Port.kUSB1);

    static Timer dTimer = new Timer();    

//Motor settings
    public static void driveSettings(){
      frontRightMotor.configSupplyCurrentLimit(configTalonCurrent);
      frontLeftMotor.configSupplyCurrentLimit(configTalonCurrent);
      backLeftMotor.configSupplyCurrentLimit(configTalonCurrent);
      backRightMotor.configSupplyCurrentLimit(configTalonCurrent);
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
    public static void driveinTime(double time, double power){
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
  public static CommandBase C_driveinTime(double time, double power){
    return new InstantCommand(
      () -> {
        driveinTime(time, power);
      }
    );
  }
    public void motorTest(){

      double magic = testMotor.getSelectedSensorPosition() + 10240;

      while(testMotor.getSelectedSensorPosition() <= magic){
      testMotor.set(ControlMode.PercentOutput, 0.5);}
      testMotor.set(ControlMode.PercentOutput, 0);

      }
    




    public void driveRotations(double rotations){
      // rotations is how much motor rotates
      testMotor.setSelectedSensorPosition(0);// starts at 0
//getSelectedSensorPosition is your current position
      double limit = testMotor.getSelectedSensorPosition() + rotations * 2048;
      while(testMotor.getSelectedSensorPosition() <= limit){
        testMotor.set(ControlMode.PercentOutput, 0.1);//run the rive
        updateVals();//UPDATE VALUES CONSTANTLY WHILE RUNNING PROGRAM
      }
         testMotor.setSelectedSensorPosition(0);
         testMotor.set(ControlMode.PercentOutput, 0.0);// sets the encoder ticks to zero
        

      }
  

    public void updateVals() {

     Constants.encoderPos = (backLeftMotor.getSelectedSensorPosition() / 2048) * 360;
     //every revolution on the motor is now worth 360, 
      double encoderVel = (backLeftMotor.getSelectedSensorVelocity() / 2048) * 360 * 10;
      Constants.error = Constants.wanted - Constants.encoderPos;


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
      SmartDashboard.putNumber("tick", driveTrain.testMotor.getSelectedSensorPosition());

      
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

    public void diveRotations(int i) {
    }
    
  //if you want to add anything, make other functions to use                                                                                                                                  

}


 
