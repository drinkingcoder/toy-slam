import processing.core.*;

public class KeyHandler {
  
  PApplet parent;
  HashMap<Integer, Boolean> keymap;

  public KeyHandler(PApplet parent) {
    this.parent = parent;
    keymap = new HashMap<Integer, Boolean>();
  }
  
  public void press(int kc) {
    keymap.put(kc, true);
  }
  
  public void release(int kc) {
    keymap.put(kc, false);
  }
  
  public boolean test(int kc) {
    if(keymap.containsKey(kc)) {
      return keymap.get(kc);
    } else {
      return false;
    }
  }
};