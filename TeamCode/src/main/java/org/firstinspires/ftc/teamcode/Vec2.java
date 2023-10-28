package org.firstinspires.ftc.teamcode;

/**
 * Ported from https://github.com/RepComm/vec2d/blob/master/src/vec.ts
 *
 */
public class Vec2 {
  public float x;
  public float y;
  
  public Vec2 (float x, float y) {
    this.set(x, y);
  }
  
  public Vec2 () {
    this(0,0);
  }
  
  public Vec2 set(float x, float y) {
    this.x = x;
    this.y = y;
    return this;
  }
  
  /**Uses other vector's same named components, returns self*/
  public Vec2 add(Vec2 other) {
    this.x += other.x;
    this.y += other.y;
    return this;
  }
  /**Uses other vector's same named components, returns self*/
  public Vec2 sub(Vec2 other) {
    this.x -= other.x;
    this.y -= other.y;
    return this;
  }
  /**Uses other vector's same named components, returns self*/
  public Vec2 div(Vec2 other) {
    this.x /= other.x;
    this.y /= other.y;
    return this;
  }
  /**Uses other vector's same named components, returns self*/
  public Vec2 mul(Vec2 other) {
    this.x *= other.x;
    this.y *= other.y;
    return this;
  }
  public Vec2 copy(Vec2 other) {
    this.set(other.x, other.y);
    return this;
  }
  public Vec2 divScalar(float scalar) {
    this.x /= scalar;
    this.y /= scalar;
    return this;
  }
  public Vec2 mulScalar(float scalar) {
    this.x *= scalar;
    this.y *= scalar;
    return this;
  }
  public float magnitude() {
    return (float) Math.sqrt(
      Math.pow(this.x, 2) +
      Math.pow(this.y, 2)
    );
  }
  public Vec2 normalize() {
    this.divScalar(this.magnitude());
    return this;
  }
  public float distance(Vec2 other) {
    return (float)(
      Math.sqrt(
        Math.pow( this.x - other.x , 2 ) +
          Math.pow( this.y - other.y , 2 )
      )
    );
  }
  public Vec2 lerp (Vec2 other, float by) {
    this.set(
      MathEx.lerp(this.x, other.x, by),
      MathEx.lerp(this.y, other.y, by)
    );
    return this;
  }
  public Vec2 centerOf (Vec2 ...vecs) {
    this.set(0,0);
    for (Vec2 other : vecs) {
      this.add(other);
    }
    this.divScalar(vecs.length);
    return this;
  }
  
  //added for simplicity, not in vec.ts original source
  public float arctan () {
    float m = this.magnitude();
    float sx = this.x / m;
    float sy = this.y / m;
    return (float) Math.atan( sy / sx);
  }
  
  public float arctan2 () {
    float m = this.magnitude();
    float sx = this.x / m;
    float sy = this.y / m;
    return (float) Math.atan2( sy, sx );
  }
}
