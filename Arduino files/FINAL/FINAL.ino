
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <math.h>
#include <Servo.h>  

#define NUM_SERVOS 7  // Number of joints

float theta1 =0;
float theta2 = 0;

float theta3 = 0;
float theta4 = 0;
float theta5 = 0;
float theta6 = 0;
// float theta7 = 0;

Servo servos[NUM_SERVOS];  // Array to store servo objects
int currentAngles[NUM_SERVOS] = {0, 90,90, 0, 90, 180, 90};  // Initial angles of servos
// int targetAngle[NUM_SERVOS] = {theta1, theta2, theta3, theta4, theta5, 180};  // Target angles for final position
int targetAngle[NUM_SERVOS];




// Servo pin numbers
int servoPins[NUM_SERVOS] = {3,4, 5, 6, 9, 10, 12};  

// ... [Keep all existing functions and calculations the same] ...

// Define constants
const float d1 = 5.38;
const float d2 = 0.0;
const float d3 = 11.217;
const float d4 = 12.876;
const float d5 = 4.4;
const float d6 = 0.0;

float cosineLaw(float a, float b, float c) {
    float h = acos((a * a + b * b - c * c) / (2 * a * b));
    // if(h<0){
    //   h = 180+h;
    // }
    return h;
}




// Function to calculate the rotation matrix
void ROTATION_MATRIX(float roll, float pitch, float yaw, float R0_6[3][3]) {


        // Initial rotation matrix
        float R0_6_initial[3][3] = {
          {0, 0, 1},
          {0, -1, 0},
          {1, 0, 0}
        };
      // Rotation matrix for roll, pitch, and yaw
      float R_6_rotation[3][3] = {
        {cos(pitch) * cos(roll),
        sin(pitch) * sin(yaw) - sin(roll) * cos(pitch) * cos(yaw),
        sin(pitch) * cos(yaw) + sin(roll) * sin(yaw) * cos(pitch)},

        {sin(roll),
        cos(roll) * cos(yaw),
        -sin(yaw) * cos(roll)},

        {-sin(pitch) * cos(roll),
        sin(pitch) * sin(roll) * cos(yaw) + sin(yaw) * cos(pitch),
        -sin(pitch) * sin(roll) * sin(yaw) + cos(pitch) * cos(yaw)}
      };

      // Compute the final rotation matrix R0_6 = R0_6_initial * R_6_rotation
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          R0_6[i][j] = 0;
          for (int k = 0; k < 3; k++) {
            R0_6[i][j] +=    R_6_rotation[k][j] *R0_6_initial[i][k]  ;
            
          }
          //Serial.println(R0_6[i][j]);
        }
      }
};


//gripper coordinate using function
void calculateGripperPosition(float finalX, float finalY, float finalZ, float d5, float R0_6[3][3], float &Xgripper, float &Ygripper, float &Zgripper) {
    // Calculate the gripper position
    Xgripper = finalX - d5 * R0_6[0][2];
    Ygripper = finalY - d5 * R0_6[1][2];
    Zgripper = finalZ - d5 * R0_6[2][2];
}

float pointToRad(float p1, float p2) {
  if (p1 > 0 && p2 >= 0) {
    return atan(p2 / p1);
  } else if (p1 == 0 && p2 > 0) {
    return PI / 2;
  } else if (p1 < 0 && p2 >= 0) {
    return fabs(atan(p2 / p1)) + PI;
  } else if (p1 < 0 && p2 < 0) {
    return atan(p2 / p1) + 2 * PI;
  } else if (p1 > 0 && p2 < 0) {
    return fabs(atan(p2 / p1));
  } else if (p1 == 0 && p2 < 0) {
    return 3 * PI / 2;
  } else if (p1 == 0 && p2 == 0) {
    return 3 * PI / 2; // edge case
  }

  return 0.0; // fallback, in case no condition matches
}



void calculateInverseKinematics(float x , float y ,float z) {
    // Calculate r and s
    float r = sqrt(x * x + y * y);
    float s = z - d1;

    // Calculate r2
    float r2 = sqrt(pow(r - d2, 2) + s * s);

    // Calculate theta1, theta2, theta3

    theta1 = atan2(y, x);

    // theta1 = pointToRad( y, x);
   



    
    targetAngle[0] = degrees(theta1);
    //  if(targetAngle[0]<0){
    //   targetAngle[0] = targetAngle[0]+180;
    // }
    // else if(targetAngle[0]>180){
    //   targetAngle[0] = 180;
    // }


    float phi3 = cosineLaw(d3, d4, r2);
     theta3 = phi3-PI;

      //theta3 = (2*PI)-phi3;
    //  theta3 = acos(((r*r)+(s*s)-(d3*d3)-(d4*d4))/(2*d3*d4));
    
    

    targetAngle[3] = degrees(theta3);
    if(targetAngle[3]<0){
      targetAngle[3] = 180-(targetAngle[3]+130);
    }
    // else if(targetAngle[3]>180){
    //   targetAngle[3] = 180;
    // }
    float phi1 = atan(s / (r - d2));
    float phi2 = cosineLaw(d3, r2, d4);
    theta2 = phi1 + phi2;
    targetAngle[1] = degrees(theta2);
     targetAngle[2] = 180-targetAngle[1];

    // if(targetAngle[1]<0){
    //   targetAngle[1] = targetAngle[1]*(-1);
    //    targetAngle[2] = 180
    // -targetAngle[1];
    // }
    // else if(targetAngle[1]>180){
    //   targetAngle[1] = 180;
    // }
   
    
}



// define roation matrix (R0_1,R1_2,R2_3)for R0_3 MATRIX 

// Define matrix types using BLA library
typedef BLA::Matrix<3, 3, float> myMatrix;



    

void INVERSE_KINEMATICS_LAST_3(float  R0_6[3][3]) {
    // Compute individual rotation matrices

     
    // Rotation matrix around the z-axis FRAME 1 
    BLA::Matrix<3 ,3 > myMatrix_M1 = {1, 0, 0,0, 0, -1,0, 1, 0};
    BLA::Matrix<3,3 > myMatrix_Rz1 = {cos(theta1), -sin(theta1), 0,sin(theta1), cos(theta1), 0, 0, 0, 1};
    BLA::Matrix<3,3 > one =  myMatrix_Rz1 * myMatrix_M1;
    
    // Rotation matrix around the z-axis
    BLA::Matrix<3 ,3 > myMatrix_M2 = {1, 0, 0,0, 1, 0,0, 0, 1};
    BLA::Matrix<3,3 > myMatrix_Rz2 = {cos(theta2), -sin(theta2), 0,sin(theta2), cos(theta2), 0,0, 0, 1};
    BLA::Matrix<3,3 > two =  myMatrix_Rz2 * myMatrix_M2;

    BLA::Matrix<3 ,3 > myMatrix_M3 = {0, 0, 1,1, 0, 0,0, 1, 0};
    BLA::Matrix<3,3 > myMatrix_Rz3 = {cos(theta3), -sin(theta3), 0,sin(theta3), cos(theta3), 0,0, 0, 1};
    BLA::Matrix<3,3 > third =  myMatrix_Rz3 * myMatrix_M2;


    

    
    // Calculate R0_3
    BLA::Matrix<3,3> R0_3 = one * (two * third);  
     
    BLA::Matrix<3, 3> R0_3_new;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            // Shift columns: column 1 -> column 2, column 2 -> column 3, column 3 -> column 1
            R0_3_new(i, (j + 2) % 3) = R0_3(i, j);
        }
    }




   

    // Compute the inverse of R0_3                      
    BLA::Matrix<3,3 > R0_3inv = Inverse(R0_3_new);


    
                                                           
    BLA::Matrix<3, 3> R0_6M;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R0_6M(i, j) = R0_6[i][j];
             Serial.print(R0_6[i][j]);
        } 
         Serial.println("  ");
    }

    

    // Calculate R3_6
    BLA::Matrix<3,3 > R3_6 = R0_3inv * R0_6M;

    // for(int i=0;i<3;i++){
    //   for(int j=0;j<3;j++){
    //     Serial.print(R3_6(i,j));
    //   }
    //   Serial.println(" ");
    // }
    
    
    
    
    
    // Calculate theta5
    theta5 = acos(R3_6(2, 2));
    targetAngle[5] = degrees(theta5);
    // if(targetAngle[4]<0){
    //   targetAngle[4] = targetAngle[4]+180;
    // }



    if (abs(sin(theta5)) > 1e-3) {
    theta6 = asin(R3_6(2, 1) / sin(theta5));
    targetAngle[6] = degrees(theta6);
    // if(targetAngle[5]<0){
    //   targetAngle[5] = targetAngle[5]+180;
    
    //  }
    targetAngle[6] = degrees(theta6);
    theta4 = asin(R3_6(1, 2) / sin(theta5));
    targetAngle[4] = degrees(theta4);
    // if(targetAngle[4]<0){
    //   targetAngle[4] = degrees(theta4)+180;
    // }
     }
    else {
     theta4 = atan2(R3_6(1,0), R3_6(1,1)); // Alternate calculation
     targetAngle[4] = degrees(theta4);
     }

  }


void run(float x,float y,float z){
     
      // Example R0_6 matrix (3x3)
        float R0_6[3][3];

        float roll = 0.0*M_PI/180;
        float pitch = 0.0*M_PI/180;
        float yaw = 0.0*M_PI/180;

        // Variables to hold the results
        float Xgripper, Ygripper, Zgripper;
        // call function for rotation matrix
        ROTATION_MATRIX(roll,  pitch,  yaw,  R0_6);
        // Call the function
        calculateGripperPosition(x, y, z, d5, R0_6, Xgripper, Ygripper, Zgripper);

        // call function for first three angle
        calculateInverseKinematics(Xgripper, Ygripper, Zgripper);

        // call function for last three angle
        INVERSE_KINEMATICS_LAST_3(R0_6);
        for (int i = 0; i < NUM_SERVOS; i++) {
          servos[i].attach(servoPins[i]);  // Attach servos to their respective pins 
          servos[i].write(currentAngles[i]);  // Set initial angles

          Serial.println("Target Angles: ");
          for(int i = 0; i < 7; i++)
            {
            Serial.print(targetAngle[i]);
            Serial.print(" ");
            }
          Serial.println(" ");
        }

        loop1();
}



void loop1() {
  // Move to target position
  bool allReached = false;
  while (!allReached) {
    allReached = true;
    for (int i = 0; i < NUM_SERVOS; i++) {

      if (currentAngles[i] < targetAngle[i]) {
        currentAngles[i]++;
        allReached = false;
      } else if (currentAngles[i] > targetAngle[i]) {
        currentAngles[i]--;
        allReached = false;
      }
      servos[i].write(currentAngles[i]);
    }
    
    
    delay(200);
  }
 
  servos[6].write(180);
  Serial.println("reached");
  servos[5].write(degrees(theta5)-30 );
  delay(1000);
  servos[6].write(90);
  Serial.println("Target position reached!");
  delay(2000);

  // Set target back to initial position
  
  for (int i = 0; i < NUM_SERVOS; i++) {
    currentAngles[i] = targetAngle[i];


  }

}

//   // Move back to initial position
//   allReached = false;
//   while (!allReached) {
//     allReached = true;
//     for (int i = 0; i < NUM_SERVOS; i++) {
//       if (currentAngles[i] < targetAngle[i]) {
//         currentAngles[i]++;
//         allReached = false;
//       } else if (currentAngles[i] > targetAngle[i]) {
//         currentAngles[i]--;
//         allReached = false;
//       }
//       servos[i].write(currentAngles[i]);
//     }
//     printAngles();
//     delay(200);
//   }
//    servos[6].write(180);
//   Serial.println("reached");
//   delay(1000);
//   servos[6].write(90);
//   Serial.println("Target position reached!");

//   Serial.println("Initial position reached!");
//   while(true); // Stop after completing cycle
// }
void loop2(){
     
  bool allReached = false;
  int initialAngles[NUM_SERVOS] = {0, 90,90, 0, 90, 180, 90};
  for (int i = 0; i < NUM_SERVOS; i++) {
    targetAngle[i] = initialAngles[i];


  }

  // Move back to initial position
  allReached = false;
  while (!allReached) {
    allReached = true;
    for (int i = 0; i < NUM_SERVOS; i++) {
      if (currentAngles[i] < targetAngle[i]) {
        currentAngles[i]++;
        allReached = false;
      } else if (currentAngles[i] > targetAngle[i]) {
        currentAngles[i]--;
        allReached = false;
      }
      servos[i].write(currentAngles[i]);
    }
   
    delay(200);
  }
  //  servos[6].write(180);
  // Serial.println("reached");
  // delay(1000);
  // servos[6].write(90);
  // Serial.println("Target position reached!");

  Serial.println("Initial position reached!");
  while(true); // Stop after completing cycle
}






void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600);
  run(10,10,2);
  delay(500);
  run(0,12,2);
  loop2();


}

void loop() {
  // put your main code here, to run repeatedly:

}
