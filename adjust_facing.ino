//hopefully Jash has these functions: turn left/right by 90 degrees
/*void turn_left()
{

}
void turn_right()
{

}
*/


int current_facing = 1;

void adjust_facing(int default_facing)
{
  //1 - north
  //2 - west
  //3 - south 
  //4 - east

  int facing_difference = current_facing - default_facing;
  switch (facing_difference)
  {
    case -3:
      turn_right();
      break;
    case -2:
      turn_right();
      turn_right();
      break;
    case -1:
      turn_left();
      break;
    case 0:
      break;
    case 1:
      turn_right();
      break;
    case 2:
      turn_right();
      turn_right();
      break;
    case 3:
      turn_left();
      break;
  } 
  
  current_facing = default_facing;

}