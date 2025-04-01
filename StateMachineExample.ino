#define STATE_OUTSIDE     0
#define STATE_INSIDE      1

bool isRain;

void stateMachine() {
  static int activityState = 0;

  switch (activityState){
    case STATE_OUTSIDE:
      //Trigger outside actions

      //State transition logic
      if (isRain) {
        activityState = STATE_INSIDE;
      } else {
        activityState = STATE_OUTSIDE;
      }
      break;

    case STATE_INSIDE:
      //Trigger inside actions

      //State transition logic
      if (isRain) {
        activityState = STATE_INSIDE;
      } else {
        activityState = STATE_OUTSIDE;
      }
      break;

    default: // error handling
      {
        activityState = STATE_OUTSIDE;
      }
      break;

  }
}