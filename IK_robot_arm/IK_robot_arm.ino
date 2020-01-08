//Inverse kinematics Robot arm with 3 degrees of freedom 
//_________________________________________________________________________________________________________________________________________________________________________

#include<Servo.h>
#include<math.h>

//_________________________________________________________________________________________________________________________________________________________________________
//Pin and arm length definitions


#define x_pin A1
#define y_pin A2
#define z_pin A3
#define switch_pin_1 4
#define servo_pin_1 5
#define servo_pin_2 6
#define servo_pin_3 7
float b_length = 5.5, f_length = 9.35;
float current_x, current_y, current_z;
float previous_x, previous_y, previous_z;
float increment = 0; // put this to zero if you want to use joysticks
float pi = 3.14159;
Servo servo_1, servo_2, servo_3;

struct var_5 // to return multiple variables
  {
    float var1, var2, var3, var4, var5;
  };
//_________________________________________________________________________________________________________________________________________________________________________
//setup


void setup() {
  Serial.begin(9600);                                              //begin serial
  
  float reach = pow(pow((f_length + b_length),2),0.5);
  Serial.println("reach");
  Serial.print(reach);
  
  current_x = 8; 
  current_y = 0;
  current_z = 5;     //Move the arm to the initial position
  
  delay(100);
  previous_x = 8; 
  previous_y = 0;
  previous_z = 5;  //And say that position before t = 0 was also the same
  
  servo_1.attach(servo_pin_1);                                     //attach servos to their respective pins
  servo_2.attach(servo_pin_2);
  servo_3.attach(servo_pin_3);
}

//_________________________________________________________________________________________________________________________________________________________________________
//Function to find the intersection of two circles

var_5 circle_intersection(float f1 ,float f2,float o1 ,float o2 ,float l1 ,float l2 )
  { var_5 points;
    
    
    float R = pow((f1*f1 + f2*f2),(0.5));
    
    float x_cor = f1 - o1;
    float y_cor = f2 - o2;
    
    
    float rot1 = (l1*l1 - l2*l2 + R*R)/(2*R);
    float rot2 = pow((l1*l1 - rot1*rot1),0.5);
    
    float x1 = ((rot1/R) * (x_cor)) + ((rot2/R) * (y_cor)) + o1;
    float y1 = ((rot1/R) * (y_cor)) - ((rot2/R) * (x_cor)) + o2;


    float x2 = ((rot1/R) * (x_cor)) - ((rot2/R) * (y_cor)) + o1;
    float y2= ((rot1/R) * (y_cor)) + ((rot2/R) * (x_cor)) + o2;
    
    Serial.print("x1:");
    Serial.println(x1); 
        
    Serial.print("x2:");
    Serial.println(x2); 

    Serial.print("y1:");
    Serial.println(y1); 
        
    Serial.print("y2:");
    Serial.println(y2); 


    points.var1 = x1;
    points.var2 = y1;
    points.var3 = x2;
    points.var4 = y2;

    return points;  //Returns two points, because almost every circle has two intersection points.
  }

  
//_________________________________________________________________________________________________________________________________________________________________________
//Function to create a line

float generate_line(float x1,float y1, float x2, float y2)
  {
  float slope;
  
  if (x2 != x1)
    {
        
     slope = (y2 - y1)/(x2 - x1);
     Serial.println("if called");
     Serial.print("slope:");
     Serial.println(slope);
    }
  else
    {
     slope = 100000; //some arbitrarily high slope
     Serial.println("else called");
    }
  Serial.print("slope:");
  Serial.println(slope);

  return slope;
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

var_5 error_protocol(float init_x,float init_y,float init_z)
  {
   float base_x = 0, base_y = 0, base_z = 0;

       
   float R_init = pow((pow((init_x - base_x),2) + pow((init_y - base_y),2)),0.5);
   float theta = atan((init_y - base_y)/(init_x - base_x));
   float Z_init = init_z - base_z;

   
   var_5 angles;
   
   
   angles = extension(R_init, Z_init, R_init, Z_init); // shoulder angle is var1, elbow angle var2.
   
   angles.var5 = angles.var2;
   angles.var4 = angles.var1;
   angles.var1 = theta;
   angles.var2 = angles.var4;
   angles.var3 = angles.var5;
   
   return angles;
       
  }
      

//_________________________________________________________________________________________________________________________________________________________________________
//Function to find angle between two lines

var_5 extension(float X_cord, float Y_cord, float target_X, float target_Y)
  { var_5 points;
    points = circle_intersection(target_X, target_Y, 0, 0, f_length, b_length);
    float x1 = points.var1 ;
    float x2 = points.var3;
    float y1 = points.var2;
    float y2 = points.var4;
    
    Serial.print("x1:");
    Serial.println(x1); 
        
    Serial.print("x2:");
    Serial.println(x2); 

    Serial.print("y1:");
    Serial.println(y1); 
        
    Serial.print("y2:");
    Serial.println(y2); 

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

    float bicep_m, forearm_m;

    bicep_m    = generate_line(0, 0, optimal_x, optimal_y);
    forearm_m  = generate_line(optimal_x, optimal_y, target_X, target_Y);

    float shoulder_angle = ang_between_lines(bicep_m, 0);
    float elbow_angle    = ang_between_lines(forearm_m, bicep_m);

    Serial.print("shoulder angle:");
    Serial.println(shoulder_angle);
    Serial.print("elbow angle:");
    Serial.println(elbow_angle);
    
    
    var_5 angles;

    angles.var1 = shoulder_angle;
    angles.var2 = elbow_angle;
    
    return angles;
    
  }

//_________________________________________________________________________________________________________________________________________________________________________
//Function to calculate angles of servos

var_5 Inverse_kinematics (float final_x, float final_y, float final_z, float init_x, float init_y, float init_z)
  {
    float error_check = 0.0;
    
    float servo_angle_1 = 0, servo_angle_2 = 0, servo_angle_3 = 0;
    float check_bounds = pow((pow(final_x,2) + pow(final_y,2) + pow(final_z,2)),0.5);
    bool check_floor = bool(final_z > 0);

    float base_size = abs(f_length - b_length);

    float reach = pow((f_length + b_length),2);
    var_5 servo_angles;

   
    if (check_bounds > base_size and check_bounds < reach and check_floor and final_y >= 0)
      {
        float base_x = 0, base_y = 0, base_z = 0;

        float R_init = pow((pow((init_x - base_x),2) + pow((init_y - base_y),2)),0.5);
        
        float R_fin  = pow((pow((final_x - base_x),2) + pow((final_y - base_y),2)),0.5);

        float theta = atan((final_y - base_y)/(final_x - base_x));

        float Z_init = init_z - base_z;
        float Z_fin  = final_z - base_z;

        var_5 angles;
        angles = extension(R_init, Z_init, R_fin, Z_fin);

        
        servo_angle_1 = theta;
        servo_angle_2 = angles.var1;
        servo_angle_3 = angles.var2;

        
        servo_angles.var1 = servo_angle_1;
        servo_angles.var2 = servo_angle_2;
        servo_angles.var3 = servo_angle_3;
        servo_angles.var4 = error_check;

      }
    else if (check_bounds < base_size)
      {
        Serial.println("Action terminated, collision with base");
        servo_angles = error_protocol(init_x, init_y, init_z);
        servo_angles.var4 = 1.0;
      }
    else if ( check_bounds > reach or final_y < 0)
      {
        Serial.println("Action terminated, not enough reach");
        Serial.println(check_bounds);
        servo_angles = error_protocol(init_x, init_y, init_z);
        servo_angles.var4 = 1.0;
      }
    else if (check_floor == 0)
      {
        Serial.println("Action terminated, collision with floor");
        servo_angles = error_protocol(init_x, init_y, init_z);
        servo_angles.var4 = 1.0;
      }
    else
      {
        Serial.println("Unknown error, please terminate the program");
        servo_angles = error_protocol(init_x, init_y, init_z);
        servo_angles.var4 = 1.0;
      }
    return servo_angles;
  }


//_________________________________________________________________________________________________________________________________________________________________________
//loop function

void loop() {
  
  
  
  float servo_1_angle, servo_2_angle, servo_3_angle, error_check;
  
  Serial.println(current_x);
  Serial.println(current_y);
  Serial.println(current_z);
  Serial.println("");
  
  
  var_5 servo_angles;
  
  servo_angles = Inverse_kinematics( current_x, current_y, current_z, previous_x, previous_y, previous_z);
  servo_1_angle = servo_angles.var1;
  servo_2_angle = servo_angles.var2;
  servo_3_angle = servo_angles.var3;
  error_check = servo_angles.var4;
  servo_1_angle = 180/3.14159 * servo_1_angle;
  if (servo_1_angle < 0)
    {
      servo_1_angle = 180 + servo_1_angle;
    }
  else
    {
      servo_1_angle = servo_1_angle;
    }
  
  servo_2_angle = 180/3.14159 * servo_2_angle;
  if (servo_2_angle < 0)
    {
      servo_2_angle = 180 + servo_2_angle;
    }
  else
    {
      servo_2_angle = servo_2_angle;
    }
  servo_2_angle = 180 - servo_2_angle;
  
  servo_3_angle = 180/3.14159 * servo_3_angle;
  if (servo_3_angle < 0)
    {
      servo_3_angle = 180 + servo_3_angle;
    }
  else
    {
      servo_3_angle = servo_3_angle;
    }
  

  
  error_check = int(error_check);

  
  float joystick_1 = 0, joystick_2 = 0, joystick_3 = 0;
  

  joystick_1 = analogRead(x_pin);
  joystick_2 = analogRead(y_pin);
  joystick_3 = analogRead(z_pin);
  delay(10);
  
  joystick_1 = (joystick_1/1023);
  joystick_2 = (joystick_2/1023);
  joystick_3 = (joystick_3/1023);
 
  if (joystick_1 < 0.2 and joystick_1 >-0.2)
    {
      joystick_1 = 0; 
    }

  if (joystick_2 < 0.2 and joystick_2 >-0.2)
    {
      joystick_2 = 0; 
    }

  if (joystick_3 < 0.3 and joystick_3 >-0.3)
    {
      joystick_3 = 0; 
    }

    
  if (error_check == 0)
   {
     for( int i = 0; i <= servo_1_angle; i++);
     {
      servo_1.write(servo_1_angle);
      delay(100);
     }
     
     for( int i = 0; i <= servo_2_angle; i++);
     {
      servo_2.write(servo_2_angle);
      delay(100);
     }
     
     for( int i = 0; i <= servo_3_angle; i++);
     {
      servo_3.write(servo_3_angle);
      delay(100);
     }

     current_x = increment*joystick_1 + current_x;
     current_y = increment*joystick_2 + current_y;
     current_z = increment*joystick_3 + current_z;

 
     previous_x = current_x;
     previous_y = current_y;
     previous_z = current_z;
   }
  else
   {
     previous_x = current_x;
     previous_y = current_y;
     previous_z = current_z;

     current_x = current_x - 1;
     current_y = current_y - 1;
     Serial.print("ERROR");
     
     for( int i = 0; i <= servo_1_angle; i++);
     {
      servo_1.write(servo_1_angle);
      delay(100);
     }
     
     for( int i = 0; i <= servo_2_angle; i++);
     {
      servo_2.write(servo_2_angle);
      delay(100);
     }
     
     for( int i = 0; i <= servo_3_angle; i++);
     {
      servo_3.write(servo_3_angle);
      delay(100);
     }
     
     
     
     
     
     
   }
  
Serial.print("servo 1's angle:");
Serial.println(servo_1_angle);
Serial.print("servo 2's angle:");
Serial.println(servo_2_angle);
Serial.print("servo 3's angle:");
Serial.println(servo_3_angle);
delay(1000000);
}
