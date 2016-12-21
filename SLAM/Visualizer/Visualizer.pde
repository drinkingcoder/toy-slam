import java.util.Map;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import hypermedia.net.*;

UDP udp;

CameraController camctrl;
KeyHandler keyhandler;
PointCloud pointcloud;

double toDouble(byte[] bytes, int start) {
  return ByteBuffer.wrap(bytes, start, 8).order(ByteOrder.LITTLE_ENDIAN).getDouble();
}

void receive(byte[] data) {
  if(data[0]==-1) {
    pointcloud.clear();
  } else if(data[0]==1) {
    PVector v = new PVector();
    v.x = (float)toDouble(data, 1);
    v.y = (float)toDouble(data, 9);
    v.z = (float)toDouble(data, 17);
    pointcloud.add(v);
  }
}

void setup() {
  size(960, 540, P3D);

  camctrl = new CameraController(this);
  keyhandler = new KeyHandler(this);
  pointcloud = new PointCloud(this);

  udp = new UDP(this, 6000);
  udp.log(false);
  udp.listen(true);
  println("UDP start listening...");
}

void draw_axis() {
  strokeWeight(0.1);
  stroke(96);
  for(int i=-10;i<=10;++i) {
    if(i==0) {
      line(-10, 0, i, 0, 0, i);
      line(i, 0, -10, i, 0, 0);
    } else {
      line(-10, 0, i, 10, 0, i);
      line(i, 0, -10, i, 0, 10);
    }
  }
  stroke(255, 0, 0);
  line(0, 0, 0, 10, 0, 0);
  stroke(0, 255, 0);
  line(0, 0, 0, 0, 10, 0);
  stroke(0, 0, 255);
  line(0, 0, 0, 0, 0, 10);
}

void keyPressed() {
  keyhandler.press(keyCode);
}

void keyReleased() {
  keyhandler.release(keyCode);
}

void mouseDragged() {
  camctrl.target.x -= (mouseX-pmouseX)/100.0;
  camctrl.target.z += (mouseY-pmouseY)/100.0;
}

float camera_y = -100;
void mouseWheel(MouseEvent event) {
  float e = event.getCount();
  camera_y -= e;
}

void handleKeys() {
  if(keyhandler.test(UP)) {
    camctrl.target.z += 1;
  }
  if(keyhandler.test(DOWN)) {
    camctrl.target.z -= 1;
  }
  if(keyhandler.test(LEFT)) {
    camctrl.target.x -= 1;
  }
  if(keyhandler.test(RIGHT)) {
    camctrl.target.x += 1;
  }
  if(keyhandler.test(KeyEvent.VK_PAGE_UP)) {
  }
}

void draw() {
  background(0, 0, 0);
  handleKeys();
  camctrl.setEyeRelative(-(mouseX-width/2)/10, camera_y, (mouseY-height/2)/10);
  camctrl.setupCamera();

  draw_axis();
  stroke(255);
  strokeWeight(0.3);
  pointcloud.draw();
}