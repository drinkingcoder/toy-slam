import processing.core.*;

public class CameraController {
  PApplet parent;
    
  public PVector target;
  public PVector eye;
  public PVector up;
  
  float internal_scale = 10;
  
  public CameraController(PApplet parent) {
    this.parent = parent;
    
    target = new PVector();
    eye = new PVector();
    up = new PVector();
    
    target.x = target.y = target.z = 0;
    eye.x = eye.y = eye.z = 0;
    up.x = up.y = up.z = 0;
    
    eye.y = -internal_scale;
    up.y = 1;
    up.z = 1;
  }
  
 
  public void setupCamera() {
    parent.perspective();
    parent.camera(eye.x*internal_scale, eye.y*internal_scale, -eye.z*internal_scale, target.x*internal_scale, target.y*internal_scale, -target.z*internal_scale, up.x, up.y, up.z);
    parent.scale(internal_scale, internal_scale, -internal_scale);
  }
    
  public void setEyeRelative(float x, float y, float z) {
    eye = target.copy();
    eye.x += x/internal_scale;
    eye.y += y/internal_scale;
    eye.z += z/internal_scale;
  }
};