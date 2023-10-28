package org.firstinspires.ftc.teamcode;

public class Omnidrive {
  
  public float[] motorAnglesRadians;
  
  public float[] motorOutput;
  
  private int motorCount;
  
  /**Create a drive with a certain number of motors equally rotated around 360deg*/
  public Omnidrive (int motorCount) {
    this.motorCount = motorCount;
    
    //allocate space for angles of motors
    this.motorAnglesRadians = new float[motorCount];
    
    //allocate space for motor torques that are calculated in 'update' function
    this.motorOutput = new float[motorCount];
    
    //loop over every motor index
    for (int i = 0; i<motorCount; i++) {
      
      //convert integers to float to preserve precision in math
      float i_f = (float) i;
      float motorCount_f = (float) motorCount;
      
      //between 0 and 1
      float factor = i_f / motorCount_f;
      
      //degrees around a circle this motor is mounted
      float degrees = factor * 360;
      
      //convert to radians
      float radians = MathEx.DEG2RAD * degrees;
      
      radians += MathEx.DEG2RAD * 90;
      
      //set motor angle
      this.setMotorAngleRadians(
        i, radians
      );
    }
  }
  
  public void setMotorAngleRadians(int index, float angleRadians) {
    this.motorAnglesRadians[index] = angleRadians;// clampRadians(angleRadians);
  }
  
  public static float motorFactor (float directionRadians, float motorRadians) {
    float diff = directionRadians - motorRadians;
    
    return (float) Math.sin(diff);
  }
  
  public void update (float directionRadians, float magnitude) {
    for (int i=0; i<this.motorCount; i++) {
      this.motorOutput[i] = motorFactor(directionRadians, this.motorAnglesRadians[i]) * magnitude;
    }
  }
}
