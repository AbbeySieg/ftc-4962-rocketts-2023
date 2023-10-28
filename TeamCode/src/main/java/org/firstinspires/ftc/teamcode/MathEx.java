package org.firstinspires.ftc.teamcode;

public class MathEx {
  public static float DEG2RAD = (float)Math.PI / 180;
  public static float lerp (float from, float to, float by) {
    return from*(1-by)+to*by;
  }
  public static float wrap (float value, float max) {
    return value % max;
  }
  public static int boolToInt (boolean v) {
    return v ? 1 : 0;
  }
}
