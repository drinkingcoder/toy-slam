import processing.core.*;

public class PointCloud {
  
  PApplet parent;
  ArrayList<PVector> points;
  
  PointCloud(PApplet parent) {
    this.parent = parent;
    points = new ArrayList<PVector>();
  }
  
  void add(PVector vec) {
    synchronized(points) {
      points.add(vec);
    }
  }
  
  void clear() {
    synchronized(points) {
      points.clear();
    }
  }
  
  void draw() {
    synchronized(points) {
      for(PVector p : points) {
        parent.point(p.x, p.y, p.z);
      }
    }
  }
}