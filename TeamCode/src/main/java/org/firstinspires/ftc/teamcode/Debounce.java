package org.firstinspires.ftc.teamcode;

public class Debounce {
  public long lastTime;
  public long minTime;
  
  public Debounce(long minTime) {
    this.minTime = minTime;
  }
  public Debounce() {
    this(100);
  }
  
  public boolean update () {
    long nowTime = System.currentTimeMillis();
    if (nowTime - this.lastTime >= this.minTime) {
      this.lastTime = nowTime;
      return true;
    }
    return false;
  }
}
