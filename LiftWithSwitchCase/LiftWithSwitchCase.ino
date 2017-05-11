
/*

  Désirée J 170512

  StateMachine - Switch cases using String

*/


//Define the different Switch-Cases
enum States {Start, GrabObject, LiftUp, ReleaseObject, LiftDown};

void setup() {
  Serial.begin(115200);
}

void loop() {

  States current_state = Start;
  States next_state;

  while (1) {
    switch (current_state) {
      case Start:
        Serial.print("Case Start\n");
        next_state = GrabObject;
        break;

      case GrabObject:
        Serial.print("Case GrabObject\n");
        next_state = LiftUp;
        break;

      case LiftUp:
        Serial.print("Case LiftUp\n");
        next_state = ReleaseObject;
        break;

      case ReleaseObject:
        Serial.print("Case ReleaseObject\n");
        next_state = LiftDown;
        break;

      case LiftDown:
        Serial.print("Case LiftDown\n");
        next_state = GrabObject;
        break;
    }

    current_state = next_state; //Change case
  }

}
