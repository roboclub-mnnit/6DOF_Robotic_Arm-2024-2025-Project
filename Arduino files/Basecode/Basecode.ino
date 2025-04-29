#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <math.h>

//using namespace BLA;



// Define constants
const float d1 = 4.3;
const float d2 = 11.5;
const float d3 = 13.1;
const float d4 = 5.2;
const float d5 = 4.4;
const float d6 = 0.0;



float cosineLaw(float a, float b, float c) {
    return acos((a * a + b * b - c * c) / (2 * a * b));
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



void calculateInverseKinematics(float x , float y ,float z ,float &theta1 , float &theta2 , float &theta3) {
    // Calculate r and s
    float r = sqrt(x * x + y * y);
    float s = z - d1;

    // Calculate r2
    float r2 = sqrt(pow(r - d2, 2) + s * s);

    // Calculate theta1, theta2, theta3
    theta1 = atan2(y, x);
    float phi3 = cosineLaw(d3, d4, r2);
    theta3 = phi3 - PI;
    float phi1 = atan(s / (r - d2));
    float phi2 = cosineLaw(d3, r2, d4);
    theta2 = phi1 + phi2;

    Serial.print("Theta 1: ");
    Serial.println(theta1*180/M_PI);
    Serial.print("Theta 2: ");
    Serial.println(theta2*180/M_PI);
    Serial.print("Theta 3: ");
    Serial.println(theta3*180/M_PI);
}



// define roation matrix (R0_1,R1_2,R2_3)for R0_3 MATRIX 

// Define matrix types using BLA library
typedef BLA::Matrix<3, 3, float> myMatrix;



    

void INVERSE_KINEMATICS_LAST_3(float theta1, float theta2, float theta3, float  R0_6[3][3]) {
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

     
    
    
    
    
    
    // Calculate theta5
    float theta5 = acos(R3_6(2, 2));

    // Calculate theta6
    float theta6 = asin(R3_6(2, 1) / sin(theta5));

    // Calculate theta4
    float theta4 = asin(R3_6(1, 2) / sin(theta5));

    // Print results
    Serial.print("Theta 4 = ");
    Serial.println(degrees(theta4));
    Serial.print("Theta 5 = ");
    Serial.println(degrees(theta5));
    Serial.print("Theta 6 = ");
    Serial.println(degrees(theta6));
}




void setup() {
        
        Serial.begin(9600);

        // Example inputs
        float finalX = 00.0;
        float finalY = 0.0;
        float finalZ = 30.0;
        float d5 = 0.2;

        // Example R0_6 matrix (3x3)
        float R0_6[3][3];

        float roll = 25.0*M_PI/180;
        float pitch = 30.0*M_PI/180;
        float yaw = 35.0*M_PI/180;

        // Variables to hold the results
        float Xgripper, Ygripper, Zgripper;
        float theta1 ,theta2 , theta3 ;

        // call function for rotation matrix
        ROTATION_MATRIX(roll,  pitch,  yaw,  R0_6);

      for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
        float x = R0_6[i][j];
        Serial.print(x);
        Serial.print("  ");
      }
       Serial.println("");
    }

        // Call the function
        int angle1[2];
        int angle2[2];
        calculateGripperPosition(finalX, finalY, finalZ, d5, R0_6, Xgripper, Ygripper, Zgripper);

        // call function for first three angle
        calculateInverseKinematics(Xgripper, Ygripper, Zgripper, theta1 ,theta2 ,theta3);

        // call function for last three angle
        INVERSE_KINEMATICS_LAST_3(theta1, theta2, theta3, R0_6);
}
void loop (){

}
