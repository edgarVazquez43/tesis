#include "InverseKinematics.h"

// #####################################
//   KINEMATIC INVERSE FOR SEVEN VALUES
bool InverseKinematics::GetInverseKinematics(std::vector<float>& cartesian, std::vector<float>& articular)
{
  std::cout << "Trying to calculate InverseKinematics .... " << std::endl;
  if(cartesian.size() != 7)
    {
      std::cout << "InverseKinematics.->Cartesian values must be seven values! " << std::endl;
      return false;
    }
  std::cout << "InverseKinematics.->Calculating for coordinates: ";
  for (int i=0; i < 7; i++) std::cout << cartesian[i] << " ";
  std::cout << std::endl;
  
  if(!GetInverseKinematics(cartesian[0], cartesian[1], cartesian[2], cartesian[3], cartesian[4], cartesian[5], cartesian[6], articular))
    {
      std::cout << "InverseKinematics.->Cannot calculate inverse kinematics u.u Point is out of workspace." << std::endl;
      return false;
    }
  
  std::cout <<"InverseKinematics.->Calculated angles: ";
  for(size_t i=0; i< articular.size(); i++)
    std::cout << articular[i] << "  ";
  std::cout << std::endl;
  
  return true;
}




// #####################################
//   KINEMATIC INVERSE FOR SEVEN VALUES
bool InverseKinematics::GetInverseKinematics_e(std::vector<float>& cartesian, std::vector<float>& articular)
{
  std::cout << "Trying to calculate InverseKinematics .... " << std::endl;

  float x_g, y_g, z_g, roll, pitch, yaw;
  float x_wc, y_wc, z_wc;
  float x_n, y_n, z_n;

  // Auxiliar variables for angles calculate
  float phi, psi, alpha, beta, gamma;
  float r, r_p, r_pp, r13;
  float a, b, c;

  articular.resize(7);
  for(int i = 0; i<articular.size(); i++) articular[i] = 0;
  
  //Denavith-Hartemberg parameters
  float dhD[7]     = {0,       0,  0.275,   0, 0.226,  0, 0.165};
  float dhA[7]     = {0.065,   0,    0,     0,   0,    0,   0  };
  float dhAlpha[7] = {1.5708, 1.5708, -1.5708, 1.5708, -1.5708, 1.5708, 0};
  float dhTheta[7] = {0,      1.5708, -1.5708,    0,      0,      0,    0};
  tf::Quaternion q;
  tf::Transform R5_EE;
  tf::Vector3 wristPosition(0,0,0);

  x_g = cartesian[0];
  y_g = cartesian[1];
  z_g = cartesian[2];
  
  roll  = cartesian[3];
  pitch = cartesian[4];
  yaw   = cartesian[5];
    
  std::cout << "Data recived..." << std::endl;
  for(int i = 0; i < cartesian.size(); i++ )
    std::cout << cartesian[i] << std::endl;
  
  q.setRPY(roll, pitch, yaw);
  R5_EE.setIdentity();
  R5_EE.setRotation(q);

  // 0.165 means distance end-effector to WristCenter
  wristPosition[2] = -0.165;
  wristPosition = R5_EE * wristPosition; //XYZ position of the end effector

  // Using decoupling kinematic we split problem inverse position an inverse orientation 
  // p_wc is the point to calculate inverse position 
  x_wc = x_g - wristPosition[0];
  y_wc = y_g - wristPosition[1];
  z_wc = z_g - wristPosition[2];

  std::cout << "x_wc: " << x_wc << std::endl;
  std::cout << "y_wc: " << y_wc << std::endl;
  std::cout << "z_wc: " << z_wc << std::endl;

  // Calculate magnitud of vetcor OWc and proyecctions on planes XY, XZ
  r = sqrt( x_wc*x_wc + y_wc*y_wc + z_wc*z_wc );
  r_p = sqrt( x_wc*x_wc + z_wc*z_wc );            // Proyection on plane ZX
  r_pp = sqrt( x_wc*x_wc + y_wc*y_wc );           // Proyection on plane XY

  std::cout << "r: " << r << std::endl;
  std::cout << "r_p: " << r_p << std::endl;
  std::cout << "r_pp: " << r_pp << std::endl;
  
  phi = atan2(y_wc, r_p);
  std::cout << "phi: " << phi << std::endl;

  // Correct displacement between M1 and M2
  // d0 = dhA[0] --- d1 = dhD[2] --- d2 = dhD[4]
  
  // d1 = dhD[2] = l2 + l3 = 0.275
  //psi = asin(z_wc/dhD[2]);

  // Code for cosine-law 
  // d2 = dhD[4] = l4 + l5 = 0.226
  a = dhD[4];
  b = dhA[0] + (dhD[2]*cos(psi));
  c = r_pp;

  alpha = acos( (a*a - b*b - c*c)/(-2*b*c) );

  x_n = x_wc - (0.065 * sin(phi-alpha));
  y_n = y_wc - (0.065 * cos(phi-alpha));

  r13 = sqrt( x_n*x_n + y_n*y_n + z_wc*z_wc );

  gamma = acos( (r13*r13 - dhD[4]*dhD[4] - dhD[2]*dhD[2])/(-2*dhD[2]*dhD[4]) );
  
  psi = asin(z_wc/r13);
  
  articular[0] = phi - alpha;
  articular[1] = psi;
  articular[3] = 3.141592 - gamma;

  return true;
}


// #####################################
//   KINEMATIC INVERSE FOR SEVEN VALUES
bool InverseKinematics::GetInverseKinematics(float x, float y, float z, float roll, float pitch, float yaw, std::vector<float>& articular)
{

    return true;
}


// #####################################
//   KINEMATIC INVERSE FOR SEVEN VALUES
bool InverseKinematics::GetInverseKinematics(float x, float y, float z, float roll, float pitch, float yaw, float elbowAngle, std::vector<float>& articular)
{
  //T O D O :   T H I S   I S   A   V E R Y   I M P O R T A N T   T O - D O !!!!!!!!!
  //Dimensions of the arms should be taken from the robot description (urdf file in the planning/knowledge/hardware/justina.xml)
  //Values of D1, D2, D3 and D4 correspond to Denavig-Hartenberg parameters and are given in the urdf
  //In the origin tag of each joint.

  //In the urdf, in each joint, originZ corresponds to the 'D' params of Denavit-Hartenberg
  //and originX corresponds to 'A' params of DenavitHartenberg. originR are the alpha parameters ant originYaw are the Theta parameters
  //in URDF:
  //<joint name="la_2_joint" type="revolute"><origin xyz="0.0603 0 0" rpy="1.5708 0 0"/>
  //<joint name="la_3_joint" type="revolute"><origin xyz="0.0 0 0" rpy="1.5708 0 1.5708"/><!--Transformation from link2 to link3 when theta2 = 0 -->
  //<joint name="la_4_joint" type="revolute"><origin xyz="0 0 0.27" rpy="-1.5708 0 -1.5708"/>
  //<joint name="la_5_joint" type="revolute"><origin xyz="0.0 0 0" rpy="1.5708 0 0"/><!--Transformation from link4 to link5 when theta4 = 0 -->
  //<joint name="la_6_joint" type="revolute"><origin xyz="0 0 0.2126" rpy="-1.5708 0 0"/>
  //<joint name="la_7_joint" type="revolute"><origin xyz="0.0 0 0" rpy="1.5708 0 0"/><!--Transformation from link6 to link7 when theta6 = 0 -->
  //<joint name="la_grip_center_joint" type="fixed"><origin xyz="0 0 0.13" rpy="0 -1.5708 3.141592"/>

  articular.resize(7);
  for(int i=0; i<7;i++) articular[i] = 0;

  float dhD[7] = {0, 0, 0.27, 0, 0.2126, 0, 0.13};
  float dhA[7] = {0.0603, 0, 0, 0, 0, 0, 0};
  float dhAlpha[7] = {1.5708, 1.5708, -1.5708, 1.5708, -1.5708, 1.5708, -1.5708};
  float dhTheta[7] = {0, 1.5708, -1.5708, 0, 0, 0, 3.141592};
  float r, alpha, beta, gamma;

  //Desired rotation matrix from base to final actuator
  //Desired orientation of the gripper is given by roll-pitch-yaw angles
  //RPY Matrix is a rotation first, of yaw degrees over Z, then, pitch degrees over CURRENT Y, and roll degrees over CURRENT x
  //RPY is also a rotation of roll over BASE X, then pitch over BASE Y, then yaw over BASE Z
  tf::Transform R07;
  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  R07.setIdentity();
  R07.setRotation(q);

  //First, we use the desired orientation to caculate the position of the center of the wrist.
  //i.e, which position should have the center of the wrist such that, with the desired orientation rpy,
  //the final effector had the desired position xyz
  tf::Vector3 WristPosition(0,0,0);
  WristPosition[0] = dhD[6];
  WristPosition = R07 * WristPosition;
  //Now, WristPosition has the desired position for the center of the wrist
  WristPosition[0] = x - WristPosition[0];
  WristPosition[1] = y - WristPosition[1];
  WristPosition[2] = z - WristPosition[2];
  x = WristPosition[0];
  y = WristPosition[1];
  z = WristPosition[2];

  //std::cout << "InverseKinematics.->WristPos: " << WristPosition[0] << " " << WristPosition[1] << " " << WristPosition[2] << std::endl;
  //std::cout << "InverseKinematics.->XYZ before correcting dhA0: " << x << " " << y << " " << z << std::endl;
  //We correct the displacement caused by the distance dhA0
  articular[0] = atan2(y, x);
  x = x - dhA[0] * cos(articular[0]);
  y = y - dhA[0] * sin(articular[0]);
  //std::cout << "InverseKinematics.->XYZ after correcting dhA0: " << x << " " << y << " " << z << std::endl;
  r = sqrt(x*x + y*y + (z-dhD[0])*(z-dhD[0]));

  if(r >= (dhD[2] + dhD[4]))
    {
      //std::cout << "InverseKinematics.->Cannot calculate inverse kinematics u.u Point is out of workspace." << std::endl;
      return false;
    }

  alpha = atan2((z-dhD[0]), sqrt(x*x + y*y));
  gamma = acos((-dhD[2]*dhD[2] - dhD[4]*dhD[4] + r*r)/(-2*dhD[2]*dhD[4]));
  beta = asin(dhD[4] * sin(gamma) / r);

  //This solution alwasy considers only the elbow-up solution. We still need to check the elbow-down solution.
  float tunningRadiusElbow = dhD[2] * sin(beta);  //Tunning radius of the elbow
  //Elbow position w.r.t. Oelbow frame
  tf::Vector3 Pelbow;
  Pelbow[0] = 0;
  Pelbow[1] = -tunningRadiusElbow * cos(elbowAngle);
  Pelbow[2] = -tunningRadiusElbow * sin(elbowAngle);

  // //Transformación del sistema sobre el que gira el codo al sistema base
  tf::Matrix3x3 oReRot;
  tf::Vector3 oReTrans;
  oReRot[0][0] = cos(articular[0]) * cos(-alpha);
  oReRot[1][0] =sin(articular[0]) * cos(-alpha);
  oReRot[2][0] =-sin(-alpha);

  oReRot[0][1] =-sin(articular[0]);
  oReRot[1][1] =cos(articular[0]);
  oReRot[2][1] =0;

  oReRot[0][2] =cos(articular[0]) * sin(-alpha);
  oReRot[1][2] =sin(articular[0]) * sin(-alpha);
  oReRot[2][2] =cos(-alpha);

  oReTrans[0] =dhD[2] * cos(beta) * cos(alpha) * cos(articular[0]);
  oReTrans[1] =dhD[2] * cos(beta) * cos(alpha) * sin(articular[0]);
  oReTrans[2] =dhD[2] * cos(beta) * sin(alpha) + dhD[0];

  tf::Transform oRe(oReRot, oReTrans);    //Homogénea del sistema Oelbow al sistema base, asumiendo que dhA0 fuera cero

  Pelbow = oRe * Pelbow; //Transformo coordenadas de posición del codo con respecto al sistema base

  articular[0] =atan2(Pelbow[1] + dhA[0]*sin(articular[0]), Pelbow[0] + dhA[0]*cos(articular[0]));
  articular[1] =atan2(Pelbow[2] - dhD[0], sqrt(Pelbow[0] * Pelbow[0] + Pelbow[1] * Pelbow[1]));
  articular[2] =0;
  articular[3] =0;

  //Calculation of Denavit-Hartenberg transforms
  tf::Transform R40;
  R40.setIdentity();
  for(size_t i=0; i < 4; i++)
    {
      tf::Transform temp;
      temp.setOrigin(tf::Vector3(dhA[i]*cos(articular[i]), dhA[i]*sin(articular[i]), dhD[i]));
      q.setRPY(dhAlpha[i],0,articular[i] + dhTheta[i]);
      temp.setRotation(q);
      R40 = R40 * temp;
    }
  R40 = R40.inverse();

  WristPosition = R40 * WristPosition;

  articular[2] = atan2(WristPosition[1], WristPosition[0]);
  articular[3] = M_PI / 2 - atan2(WristPosition[2], sqrt(WristPosition[0] * WristPosition[0] + WristPosition[1] * WristPosition[1]));
  if(articular[3] > M_PI) articular[3] -= 2*M_PI;
  if(articular[3] <= -M_PI) articular[3] += 2*M_PI;

  R40.setIdentity();
  for(size_t i=0; i < 4; i++)
    {
      tf::Transform temp;
      temp.setOrigin(tf::Vector3(dhA[i]*cos(articular[i]), dhA[i]*sin(articular[i]), dhD[i]));
      q.setRPY(dhAlpha[i],0,articular[i] + dhTheta[i]);
      temp.setRotation(q);
      R40 = R40 * temp;
    }
  R40 = R40.inverse();

  //A partir de aquí se calculan los ángulos de orientación
  tf::Transform tfR47;    // Matriz 4R7 de orientación de la muñeca deseada implicita
  tfR47 = R40 * R07;
  tf::Matrix3x3 R47;
  R47 = tfR47.getBasis();

  if((1 - fabs(R47[2][0])) < 0.0001)
    {
      //It means the second movement of the wrist, i.e. servo5 (6th servo) has an angle of zero
      //Thus, the desired orientation can be reached with an infinity number of combinations (a singularity)
      //of servos 5th and 7th, so, it is choosen to move only the last servo
      articular[4] = 0;
      articular[5] = 0;
      articular[6] = atan2(R47[1][1], R47[0][1]);
    }
  else
    {
      articular[4] = atan2(R47[1][0], R47[0][0]);
      articular[5] = atan2(sqrt(1 - R47[2][0]*R47[2][0]), R47[2][0]);

      if(articular[4] > M_PI/2)
	{
	  articular[4] -= M_PI;
	  articular[5] *= -1;
	}
      else if(articular[4] < -M_PI/2)
	{
	  articular[4] += M_PI;
	  articular[5] *= -1;
	}
      float psi1 = atan2(R47[2][2], -R47[2][1]);
      float psi2 = atan2(-R47[2][2], R47[2][1]);
      if(fabs(psi1) < fabs(psi2))
	articular[6] = psi1;
      else
	articular[6] = psi2;
    }

  return true;
}


// #####################################
//   KINEMATIC INVERSE FOR SEVEN VALUES
bool InverseKinematics::GetInverseKinematics(float x, float y, float z, std::vector<float>& articular)
{

    return true;
}


// #####################################
//   KINEMATIC INVERSE FOR SEVEN VALUES
bool InverseKinematics::GetDirectKinematics(std::vector<float>& articular, std::vector<float>& cartesian)
{
  std::cout << "Trying to calculate DirectKinematics (Edgar Version)" << std::endl;
  
  if(articular.size() != 7)
    {
      std::cout << "Error in articular size, it must contain seven values" << std::endl;
      return false;
    }
  
  //Denavith-Hartenberg Parameters
  float dhD[7]     = {0, 0, 0.275, 0, 0.226, 0, 0.165};
  float dhA[7]     = {0.065, 0, 0, 0, 0, 0, 0};
  float dhAlpha[7] = {1.5708, 1.5708, -1.5708, 1.5708, -1.5708, 1.5708, 0};
  float dhTheta[7] = {0, 1.5708, -1.5708, 0, 0, 0, 0};
  

  tf::Quaternion q, q01, q12, q23, q34, q45, q56, q67;
  tf::Transform R01, R12, R23, R34, R45, R56, R67;
  tf::Transform R07;

  R01.setIdentity();
  R12.setIdentity();
  R23.setIdentity();
  R34.setIdentity();
  R45.setIdentity();
  R56.setIdentity();
  R67.setIdentity();
  
  R07.setIdentity();
  
  R01.setOrigin(tf::Vector3( dhA[0]*cos(articular[0]), dhA[0]*sin(articular[0]), dhD[0]) );
  R12.setOrigin(tf::Vector3( dhA[1]*cos(articular[1]), dhA[1]*sin(articular[1]), dhD[1]) );
  R23.setOrigin(tf::Vector3( dhA[2]*cos(articular[2]), dhA[2]*sin(articular[2]), dhD[2]) );
  R34.setOrigin(tf::Vector3( dhA[3]*cos(articular[3]), dhA[3]*sin(articular[3]), dhD[3]) );
  R45.setOrigin(tf::Vector3( dhA[4]*cos(articular[4]), dhA[4]*sin(articular[4]), dhD[4]) );
  R56.setOrigin(tf::Vector3( dhA[5]*cos(articular[5]), dhA[5]*sin(articular[5]), dhD[5]) );
  R67.setOrigin(tf::Vector3( dhA[6]*cos(articular[6]), dhA[6]*sin(articular[6]), dhD[6]) );
  
  q01.setRPY(dhAlpha[0], 0, articular[0] + dhTheta[0]);
  q12.setRPY(dhAlpha[1], 0, articular[1] + dhTheta[1]);
  q23.setRPY(dhAlpha[2], 0, articular[2] + dhTheta[2]);
  q34.setRPY(dhAlpha[3], 0, articular[3] + dhTheta[3]);
  q45.setRPY(dhAlpha[4], 0, articular[4] + dhTheta[4]);
  q56.setRPY(dhAlpha[5], 0, articular[5] + dhTheta[5]);
  q67.setRPY(dhAlpha[6], 0, articular[6] + dhTheta[6]);
  
  R01.setRotation(q01);
  R12.setRotation(q12);
  R23.setRotation(q23);
  R34.setRotation(q34);
  R45.setRotation(q45);
  R56.setRotation(q56);
  R67.setRotation(q67);
  
  
  //This is the transformation from baseArm to endEffector
  R07 = R01 * R12 * R23 * R34 * R45 * R56 * R67;
  
  tf::Vector3 endEffector(0,0,0);
  endEffector = R07 * endEffector; //XYZ position of the end effector
  q = R07.getRotation();
  
  
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  
  
  cartesian.clear();
  cartesian.push_back(endEffector.x());
  cartesian.push_back(endEffector.y());
  cartesian.push_back(endEffector.z());
  cartesian.push_back(roll - 1.5708); //This minus pi/2 corrects the fact that the final-effector frame is not aligned with left_arm_link0
  cartesian.push_back(pitch);
  cartesian.push_back(yaw - 1.5708);
  cartesian.push_back(0);
  
  std::cout << "DirectKinematics.->Calculated cartesian: " << std::endl;
  for (int i=0; i < 7; i++) std::cout << "   " << cartesian[i] << std::endl;
  std::cout << std::endl;
  
  return true;
}

