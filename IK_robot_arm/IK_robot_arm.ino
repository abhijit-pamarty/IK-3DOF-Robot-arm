//Inverse kinematics Robot arm with 3 degrees of freedom 
//_________________________________________________________________________________________________________________________________________________________________________

#include<Servo.h>
#include<math.h>

//_________________________________________________________________________________________________________________________________________________________________________
//Pin and arm length definitions


#define x_pin A0
#define y_pin A1
#define z_pin A2
#define switch_pin_1 4
#define servo_pin_1 5
#define servo_pin_2 6
#define servo_pin_3 7
float b_length = 7.0, f_length = 8.0;
float current_x, current_y, current_z;
float previous_x, previous_y, previous_z;
float increment = 0.01;
Servo servo_1, servo_2, servo_3;

//_________________________________________________________________________________________________________________________________________________________________________
//setup


void setup() {
  Serial.begin(9600);                                              //begin serial
  
  float reach = pow((f_length + b_length),2);
  current_x, current_y, current_z = reach/2, reach/2, reach/2;     //Move the arm to the initial position
  previous_x, previous_y, previous_z = reach/2, reach/2, reach/2;  //And say that position before t = 0 was also the same
  
  servo_1.attach(servo_pin_1);                                     //attach servos to their respective pins
  servo_2.attach(servo_pin_2);
  servo_3.attach(servo_pin_3);
}

//_________________________________________________________________________________________________________________________________________________________________________
//Function to find the intersection of two circles

float circle_intersection(float f1 = 0,float f2 = 0,float o1 = 0,float o2 = 0,float l1 = 0,float l2 = 0)
  {
    
    
    float R = pow((f1*f1 + f2*f2),(0.5));
    
    float x_cor = f1 - o1;
    float y_cor = f2 - o2;
    
    
    float rot1 = (l1*l1 - l2*l2 + R*R)/(2*R);
    float rot2 = pow((l1*l1 - rot1*rot1),0.5);
    
    float x1 = ((rot1/R) * (x_cor)) + ((rot2/R) * (y_cor)) + o1;
    float y1 = ((rot1/R) * (y_cor)) - ((rot2/R) * (x_cor)) + o2;


    float x2 = ((rot1/R) * (x_cor)) - ((rot2/R) * (y_cor)) + o1;
    float y2= ((rot1/R) * (y_cor)) + ((rot2/R) * (x_cor)) + o2;



    return x1,y1,x2,y2;  //Returns two points, because almost every circle has two intersection points.
  }

  
//_________________________________________________________________________________________________________________________________________________________________________
//Function to create a line

float generate_line(float x1,float y1, float x2, float y2)
  {
  float slope;
  
  if (x2 != x1)
    {
     slope = (y2 - y1)/(x2 - x1);
    }
  else
    {
     slope = 100000; //some arbitrarily high slope
    }
  float constant = -1*slope*x1 + y1;

  return slope, constant;
  }

//_________________________________________________________________________________________________________________________________________________________________________
//Function to find angle between two lines

float ang_between_lines(float m1,float m2)
  {
    return atan((m1-m2)/(1+(m1*m2)));
  }

//_________________________________________________________________________________________________________________________________________________________________________
//Subfunction in case of physical constraints
//Designed to maintain position

    float error_protocol(float init_x,float init_y,float init_z)
      {
       float base_x = 0, base_y = 0, base_z = 0;

       
       float R_init = pow((pow((init_x - base_x),2) + pow((init_y - base_y),2)),2);
       float theta = atan((init_y - base_y)/(init_x - base_x));
       float Z_init = init_z - base_z;
       float shoulder_angle, elbow_angle;
       shoulder_angle, elbow_angle = extension(R_init, Z_init, R_init, Z_init);

       return theta, shoulder_angle, elbow_angle;
       
      }
      

//_________________________________________________________________________________________________________________________________________________________________________
//Function to find angle between two lines

float extension(float X_cord, float Y_cord, float target_X, float target_Y)
  {
    float x1,y1,x2,y2;
    x1,y1,x2,y2 = circle_intersection(target_X, target_Y, 0, 0, f_length, b_length);
    
    //find out optimal point, that is, point closest to the current position
     
    float optimal_x, optimal_y;
    float min_distance = 10000000.0; //some arbitrary value
    
    
    float distance_p1 = pow((pow((x1 - X_cord),2) + pow((y1 - Y_cord),2)),0.5);
    float distance_p2 = pow((pow((x2 - X_cord),2) + pow((y2 - Y_cord),2)),0.5);

    if (distance_p1 >= distance_p2)
      {
        optimal_x = x2;
        optimal_y = y2;
      }
    else
      {
        optimal_x = x1;
        optimal_y = y1;
        
      }

    float bicep_m, bicep_c, forearm_m, forearm_c;

    bicep_m, bicep_c      = generate_line(0, 0, optimal_x, optimal_y);
    forearm_m, forearm_c  = generate_line(optimal_x, optimal_y, target_X, target_Y);

    float shoulder_angle = ang_between_lines(bicep_m, 0);
    float elbow_angle    = ang_between_lines(forearm_m, bicep_m);

    return shoulder_angle, elbow_angle;
    
  }

//_________________________________________________________________________________________________________________________________________________________________________
//Function to calculate angles of servos

float Inverse_kinematics (float final_x, float final_y, float final_z, float init_x, float init_y, float init_z)
  {
    float error_check = 0.0;
    
    float servo_angle_1 = 0, servo_angle_2 = 0, servo_angle_3 = 0;
    float check_bounds = pow(final_x,2) + pow(final_y,2) + pow(final_z,2);
    bool check_floor = bool(final_z > 0);

    float base_size = abs(f_length - b_length);

    float reach = pow((f_length + b_length),2);
    

   
    if (check_bounds > base_size and check_bounds < reach and check_floor and final_y >= 0)
      {
        float base_x = 0, base_y = 0, base_z = 0;

        float R_init = pow((pow((init_x - base_x),2) + pow((init_y - base_y),2)),2);
        float R_fin  = pow((pow((final_x - base_x),2) + pow((final_y - base_y),2)),2);

        float theta = atan((final_y - base_y)/(final_x - base_x));

        float Z_init = init_z - base_z;
        float Z_fin  = final_z - base_z;

        float shoulder_angle, elbow_angle;
        shoulder_angle, elbow_angle = extension(R_init, Z_init, R_fin, Z_fin);
        

        servo_angle_1 = theta;
        servo_angle_2 = shoulder_angle;
        servo_angle_3 = elbow_angle;

      }
    else if (check_bounds < base_size)
      {
        Serial.println("Action terminated, collision with base");
        servo_angle_1, servo_angle_2, servo_angle_3 = error_protocol(init_x, init_y, init_z);
        error_check = 1.0;
      }
    else if ( check_bounds > reach or final_y < 0)
      {
        Serial.println("Action terminated, not enough reach");
        servo_angle_1, servo_angle_2, servo_angle_3 = error_protocol(init_x, init_y, init_z);
        error_check = 1.0;
      }
    else if (check_floor == 0)
      {
        Serial.println("Action terminated, collision with floor");
        servo_angle_1, servo_angle_2, servo_angle_3 = error_protocol(init_x, init_y, init_z);
        error_check = 1.0;
      }
    else
      {
        Serial.println("Unknown error, please terminate the program");
        servo_angle_1, servo_angle_2, servo_angle_3 = error_protocol(init_x, init_y, init_z);
        error_check = 1.0;
      }
    return servo_angle_1, servo_angle_2, servo_angle_3, error_check;
  }


//_________________________________________________________________________________________________________________________________________________________________________
//loop function

void loop() {

  
  
  float servo_1_angle, servo_2_angle, servo_3_angle, error_check;

  servo_1_angle, servo_2_angle, servo_3_angle, error_check = Inverse_kinematics( current_x, current_y, current_z, previous_x, previous_y, previous_z);

  error_check = int(error_check);

 
  
  float joystick_1 = 0, joystick_2 = 0, joystick_3 = 0;
  

  joystick_1 = analogRead(x_pin);
  joystick_2 = analogRead(y_pin);
  joystick_3 = analogRead(z_pin);

  joystick_1 = map(joystick_1, 0, 1023, -1, 1);
  joystick_2 = map(joystick_2, 0, 1023, -1, 1);
  joystick_3 = map(joystick_3, 0, 1023, -1, 1);
  
  if (error_check == 0)
   {
     servo_1.write(servo_1_angle);
     servo_2.write(servo_2_angle);
     servo_3.write(servo_3_angle);

     current_x = increment*joystick_1 + current_x;
     current_y = increment*joystick_2 + current_y;
     current_z = increment*joystick_3 + current_z;

     previous_x = current_x;
     previous_y = current_y;
     previous_z = current_z;
   }
  else
   {
     servo_1.write(servo_1_angle);
     servo_2.write(servo_2_angle);
     servo_3.write(servo_3_angle);
      
   }
  

}
