/* Shared Control based on Policy Iteration: ROS node code
*/

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>

#include<vector>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>

#include <std_msgs/Int16MultiArray.h>   // for ultrasonic msg 
#include <std_msgs/Float32MultiArray.h> // for odometry msg

//#include <cmath> // for std::pow

class SharedControl {
private:
   double vd,omegad;
   double v;

   std::vector<double>US_reading_m;
   std::vector<double>US_reading_m_lkv; // last known valid value = lkv

   ros::NodeHandle nh_; // in C++ the naming convention of a variable with an underscore usually indicates a private member variable: see B2017Newman, Comment p55
   ros::Subscriber joy_sub;
   ros::Subscriber ultrasonic_sub; 
   ros::Subscriber odom_sub;
   ros::Publisher  vel_cmd_pub;

   // member function
   void joySubscCallback(const sensor_msgs::Joy::ConstPtr& msg) {
      //ROS_INFO("/joy: received msg: [%f,%f]", msg->axes[0],msg->axes[1]);

      float omega_joy_INSA_perc = msg->axes[0]; //[%] \in [-100,100]
      float v_joy_INSA_perc     = msg->axes[1]; //[%] \in [-100,100]

      // convert (v_joy_INSA_perc [%], omega_joy_INSA_perc [%]) into (vd[m/s], omegad[m/s])
      omegad = -(omega_joy_INSA_perc /100.0)*Profile_omegamax; //[rad/s]
      if (v_joy_INSA_perc>=0) {
         vd = (v_joy_INSA_perc/100.0)*Profile_absvmax_fwd;//[m/s]
      } else {
         vd = (v_joy_INSA_perc/100.0)*Profile_absvmax_bwd;//[m/s]
      } // if

   } // void joySubscCallback(.)

   // member function
   void ultrasonicSubscCallback(const std_msgs::Int16MultiArray::ConstPtr& msg) {
      //ROS_INFO("/ultrasonic_array: received msg:");
      for (uint8_t iUS = 0; iUS < US_totalNrSensors; iUS++) { // ROS field types (e.g int64_t,uint8_t,etc.): http://wiki.ros.org/msg
         //ROS_INFO("Sensor index %u: value %u", iUS, msg->data[iUS]);

         // // EXTRACT SIGNAL + CORRECTION
         // ini & by default assume signal is valid
         US_reading_m[iUS] = ((double)(msg->data[iUS]))/100.0; //[m]
         bool isSignalValid = true;

         // check if signal not valid
         if ((US_reading_m[iUS]<US_draymin) || (US_reading_m[iUS]>US_draymax)) { //then sth went wrong with this measurement: decision to keep the previous valid one
            US_reading_m[iUS] = US_reading_m_lkv[iUS]; //overwrite; f signal not valid: decision to use last known valid value (lkv)   
            isSignalValid = false;
         }      
         
         // store & update
         if (isSignalValid) { //then it makes sense to store & update
            US_reading_m_lkv[iUS] = US_reading_m[iUS];
         }

      } // for

   } // void ultrasonicSubscCallback(.)

   // member function
   void odometrySubscCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
      // def accordingly
      uint8_t odm_totalNrReadings = 5; // ROS field types (e.g int64_t,uint8_t,etc.): http://wiki.ros.org/msg

      //ROS_INFO("/odometry_array: received msg:");
      for (uint8_t idx = 0; idx < odm_totalNrReadings; idx++) {
         //ROS_INFO("Sensor index %u: value %u", idx, msg->data[iUS]);
      } // for

      v = (double)(msg->data[3]); // [m/s] linear velocity

      // call
      sharedControlAlgo();

   } // void odometrySubscCallback(.)

   // member function
   void sharedControlAlgo() {
      // Assumption: signals (v,vd,omegad) are perfect (no correction needed; actually there is no error flagging mechanism on these signals); US readings might indicate/flag errors and we shall use this in the shared control algo 

      // ini & by default assume case1: decision to leave user input unchanged
      double vr     = vd;     //[m/s] 
      double omegar = omegad; //[rad/s] 

      // interface
      double dmax = US_draymax; //[m]
      double vmax = Profile_absvmax_fwd; //[m/s]

      // // boundary line  
      // choose 
      double deltax = .85; //1.15; //[m] distance between xA and obstacle; deltax \in (0,dmax), deltax =def= dmax-xA 

      // conseq
      double xA = dmax - deltax; //[m] xA \in (0,US_draymax) belongs to x-axis
   
      // // algo to set dUbLok
      // by default assume situ1 i.e. both readings are valid
      double dUbLok = std::min(US_reading_m[2],US_reading_m[3]); //wcs approach based on these 2 sensors; //[m]

      // check & handle situ2 i.e. one reading is faulty
      if (dUbLok<0) dUbLok = std::max(US_reading_m[2],US_reading_m[3]);

      // check & handle situ3 i.e. both readings are faulty
      if (dUbLok<0) { //then both meas are faulty and cannot be used for SC
         ROS_INFO("Both US readings are faulty at this time instant");

         publish_velocity_cmd(vr,omegar); // publish default values
         return; // no reason to pursue SC algo
      }
      // otherwise continue hereafter

      double x = dmax - dUbLok;
      
      // [var1] def fbnd: linear fct fbnd(x)=a*x+b, intersecting the points (x=xA,v=vmax) and (x=dmax,v=0) 
      //double fbnd = -vmax*(x-dmax)/(dmax-xA); // fbnd=fbnd(x)
      // [var2] def fbnd: equivalent/same to above  
      //double fbnd = vmax*dUbLok/deltax; // fbnd=fbnd(x)

      // [var3] def fbnd: quadratic fct fbnd(x)=a*(x-dmax)^2
      double fbnd = vmax*pow(x-dmax,2)/pow(xA-dmax,2); 

      // [var4] def fbnd: Nth order fct fbnd(x)=a*(x-dmax)^N: cannot implement because pow(double,double) only supported by C++11, whereas ROS kineric is C++03
      //double N = 1.55; // choose a number: N=1 makes fbnd(x) a linear fct; N=2 makes fbnd(x) quadratic, etc.
      //double fbnd = vmax*pow(x-dmax,N)/pow(xA-dmax,N);  
      
      // case2
      //if ((x>xA) && (x<=dmax) && (v>fbnd)) { //then apply SC
      if ((dUbLok<deltax) && (v>fbnd)) { //then apply SC, equivalent if-clause as above
         // 2 situations analyzed below depending on vd 
         if (vd>fbnd) {  
            vr = std::min(vd,fbnd); // saturate vr by an upper bnd to be fbnd; handles situ where v>fbnd in virtue of physical inertia, but user desires/asks vd<fbnd 
            omegar = omegad*vr/vd;  // no pb with singu here because denominator \neq0
         } else {
            // leave (vd,omegad) unchanged
         } // if else

      } // if

      // print/display input-outputs of shared control algo   
      ROS_INFO("Shared   : Inputs:  vd=%.3f; omegad=%.3f; v=%.3f; fbnd=%.3f; dUbLok=%.3f", vd,omegad, v,fbnd, dUbLok);
      ROS_INFO("  Control: Outputs: vr=%.3f; omegar=%.3f",vr,omegar);

      publish_velocity_cmd(vr,omegar); // vr [m/s], omegar [rad/s]
   } // void sharedControlAlgo()

   // member function: helper function	
   void publish_velocity_cmd(double vr, double omegar) {
      //def&ini: ini for safety reasons
      geometry_msgs::TwistStamped vel_cmd_msg;

      // ROS
      vel_cmd_msg.header.stamp = ros::Time::now();
      vel_cmd_msg.twist.linear.x   = vr; //[m/s] 
      vel_cmd_msg.twist.angular.z  = omegar; //[rad/s]       

      vel_cmd_pub.publish(vel_cmd_msg);
   }

public:
   static const double Profile_absvmax_fwd; 
   static const double Profile_absvmax_bwd; 
   static const double Profile_omegamax; 

   static const double US_draymax;
   static const double US_draymin;
   static const uint8_t US_totalNrSensors = 12;
   
   // constructor
   SharedControl(ros::NodeHandle *nhPtr):nh_(*nhPtr) {
      // ini
      vd     = 0.0;
      omegad = 0.0;
      v      = 0.0;
      
      for (uint8_t i=0; i<US_totalNrSensors; i++) {
         const double US_faulty = -1.0; // -1.0 is used to flag an error
         US_reading_m.push_back(US_faulty); //safe ini 
         US_reading_m_lkv.push_back(US_faulty); //safe ini 
      } // for

      joy_sub = nh_.subscribe("/joy", 1, &SharedControl::joySubscCallback, this);
      ultrasonic_sub = nh_.subscribe("/ultrasonic_array", 1, &SharedControl::ultrasonicSubscCallback, this);
      odom_sub = nh_.subscribe("/odometry_array", 1, &SharedControl::odometrySubscCallback, this);
      vel_cmd_pub = nh_.advertise<geometry_msgs::TwistStamped>("/velocity_cmd", 1);

      ROS_INFO(" Launched Shared Control!\n");
   } // SharedControl(.)

   // destructor
   ~SharedControl() {
      ROS_INFO(" Ended Shared Control!\n");
   } // ~SharedControl()

   // member function
   double get_v() const {return v;}

   // member function
   void printData() const {
      ROS_INFO("v=%.3f; vd=%.3f; omegad=%.3f",v,vd,omegad);
   }

   // static class member: static member function
   static double mph2mps(double mph) {
	return mph * 0.44704;
   } 

}; // class SharedControl

// ++++++++++++++++++++++++++++++++++++++
// def: Profile ADAPT + speed=5: characteristics
const double SharedControl::Profile_absvmax_fwd = SharedControl::mph2mps(1.6); //[m/s] experimentally identified
const double SharedControl::Profile_absvmax_bwd = 0.32; //[m/s] experimentally identified
const double SharedControl::Profile_omegamax = 1.0; //[rad/s] experimentally identified

// def: ultrasonic sensors param/settings
const double SharedControl::US_draymax = 2.838; //[m] cf calibration, namely chosen 
const double SharedControl::US_draymin = 0.040; //[m] cf datasheet

// ++++++++++++++++++++++++++++++++++++++
int main(int argc, char** argv) {
   ROS_INFO("%f",SharedControl::Profile_absvmax_fwd);

   ros::init(argc,argv, "shared_control_node");
   ros::NodeHandle nh;
   SharedControl sc = SharedControl(&nh);
   ros::spin();
} // int main(.)
