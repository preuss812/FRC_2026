 java.lang.Math;

public class Slew {
   public static double slewRateLimiter(double last, double ask, double maxDecrease, double maxIncrease, double elapsedTime) {
    double next = last;
    double delta = ask - last;
    int lastSign = (int)Math.signum(last);
    int deltaSign = (int)Math.signum(delta);

    // Branch on the sign of the delta value
    switch(deltaSign) {
      case -1:
        switch(lastSign) {
          case -1:
          case 0:
	    next = Math.max(last - maxIncrease * elapsedTime, ask);
            break;
          case 1:
	    next = Math.max(last - maxDecrease * elapsedTime, ask);
            break;
          default:
            next = last; // Should never happen
            break;
        }
        break ;
      // When the delta is 0, stay with the current/last value
      case 0:
        next = last;
        break;
      case 1:
        switch(lastSign) {
          case -1:
            next = Math.min(last + maxDecrease * elapsedTime, ask);
            break;
          case 0:
          case 1:
	    next = Math.min(last + maxIncrease * elapsedTime, ask);
            break;
          default:
            next = last; // Should never happen
            break;
        }
        break;
      default:
        next = last; // Should never happen
        break;
    }

    return next;
  }

  public static void main(String[] args) {
    double x;
    double y;
    x = 0;
    y = 0;

    double lastx = 0;
    double lasty = 0;

    for (int i = 0; i <= 30; i++) {
       x = slewRateLimiter(lastx, 1.0, 2.0, 1.0, 0.02);
       System.out.printf("%d: %5.3f -> %5.2f\n", i, lastx, x);
       lastx = x;
     }
    for (int i = 0; i <= 60; i++) {
       x = slewRateLimiter(lastx, -1.0, 2.0, 1.0, 0.02);
       System.out.printf("%d: %5.3f -> %5.2f\n", i, lastx, x);
       lastx = x;
     }
    for (int i = 0; i <= 60; i++) {
       x = slewRateLimiter(lastx, 1.0, 2.0, 1.0, 0.02);
       System.out.printf("%d: %5.3f -> %5.2f\n", i, lastx, x);
       lastx = x;
     }
  }
}

  
