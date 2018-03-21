#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Float64.h>

#define PI 3.1415926535897932384626433f
#define MAX_TURN_SPD 0.42359879
#define MIN_TURN_SPD 0.05

class RobotinoOdometry
{
    ros::Publisher wheel_0;
    ros::Publisher wheel_1;
    ros::Publisher wheel_2;
    ros::NodeHandle nh;

    int onlyOnce = 0;

// DO NOT EDIT ANY VALUES IN BETWEEN THESE COMMENTS.
    double b0=PI, b1=PI, b2=PI; // beta, steering angle (angle of wheel plane relative to the robot main body)
    double l0=0.13, l1=0.13, l2=0.13; // distance from the geometrical center of the robot to the center of the wheel (wheelbase)
    double a0=PI/2, a1=7*PI/6, a2=11*PI/6; // alpha, angle between the wheel shaft and X R axis when the robot is at home position
    double g0=-PI, g1=-PI, g2=-PI; // gamma, angle between the main wheel plane and the axis of rotation of the small circumferential rollers
    double Th = 0; // theta, orientation angle
    double r0=0.04, r1=0.04, r2=0.04; // radius of the wheel
// DO NOT EDIT ANY VALUES IN BETWEEN THESE COMMENTS.

    double InverseJacobian[3][3]; // J^-1
    double CorrectedInverseJacobian[3][3]; // Jc^-1
    double Flon;
    double Flat[3][3] = {{0,0,0},
                         {0,0,0},
                         {0,0,0}};
    double error_angle_dot; // equation 9
    double angular_velocity_errors[3]; // equation 9

public:
    RobotinoOdometry()
    {
        wheel_0 = nh.advertise<std_msgs::Float64> ("/vrep/wheel_0_Command",1);
        wheel_1 = nh.advertise<std_msgs::Float64> ("/vrep/wheel_1_Command",1);
        wheel_2 = nh.advertise<std_msgs::Float64> ("/vrep/wheel_2_Command",1);

        setInvJacobian();

        /*
            TODO_2: You should calculate the error angle by doing the proposed calibration method, explained
            in detail in Figure 3 in the paper. Dividing this error angle to the time it takes for the robot 
            to finish its trajectory should be assigned to the variable 'error_angle_dot'. For example if the 
            error angle is 60 degrees = PI/3 radians and inside the main function ros rate is set to 20 (times 
            in a second) and your speed commands run for 150 iterations, t = 150/20 = 7.5. Then 
        */
        error_angle_dot = -0.0149; //This is given just for an example, change this value.
        /*    
            After setting the error_angle_dot variable to a proper value, setAngularVelocityErrorsAndFlat() 
            function is called below. Go edit that function also.
        */

        setAngularVelocityErrorsAndFlat();

        /*
            TODO_4: You should calculate the value of Flon and set it to the variable Flon.
            Then call setCorrectedInvJacobian() method to multiply Flon and Flat with the InverseJacobian matrix.
            After doing that you can edit the setSpeed() method in order to use CorrectedInverseJacobian matrix,
            instead of InverseJacobian matrix.
        */
        Flon = 0.858425; // replace this value with the value found from the equation 11 in the paper.

        // Uncomment the next line if you set Flon and Flat variables correctly.
        setCorrectedInvJacobian();
    }

    void setInvJacobian(){
        InverseJacobian[0][0] = sin(a0 + b0 + g0 + Th) / (r0 * cos(g0));
        InverseJacobian[1][0] = sin(a1 + b1 + g1 + Th) / (r1 * cos(g1));
        InverseJacobian[2][0] = sin(a2 + b2 + g2 + Th) / (r2 * cos(g2));
        InverseJacobian[0][1] = -cos(a0 + b0 + g0 + Th) / (r0 * cos(g0));
        InverseJacobian[1][1] = -cos(a1 + b1 + g1 + Th) / (r1 * cos(g1));
        InverseJacobian[2][1] = -cos(a2 + b2 + g2 + Th) / (r2 * cos(g2));
        InverseJacobian[0][2] = -l0 * cos(b0 + g0) / (r0 * cos(g0));
        InverseJacobian[1][2] = -l1 * cos(b1 + g1) / (r1 * cos(g1));
        InverseJacobian[2][2] = -l2 * cos(b2 + g2) / (r2 * cos(g2));
    }

    void setAngularVelocityErrorsAndFlat(){
        /*
            TODO_3: set angular_velocity_errors using InverseJacobian matrix and error_angle_dot variable.
            Then inside this function set the Flat matrix using angular_velocity_errors and nominal angular velocities.
            You can find the nominal angular velocities by copying the w0, w1, w2 values printed on the command line
            when you run this code.
        */
		angular_velocity_errors[0] = InverseJacobian[0][2] * -1 * error_angle_dot;
		angular_velocity_errors[1] = InverseJacobian[1][2] * -1 * error_angle_dot;
		angular_velocity_errors[2] = InverseJacobian[2][2] * -1 * error_angle_dot;

		Flat[0][0] = 1 + angular_velocity_errors[0] / -3.99 ;
		Flat[1][1] = 1 + angular_velocity_errors[1] / 2.0;
		Flat[2][2] = 1 + angular_velocity_errors[2] / 2.0;
		std::cout << "FLAT" << Flat[0][0] << " " << Flat[1][1] << " " <<  Flat[2][2] << " " << std::endl ;
		
    }

    void setCorrectedInvJacobian(){
        CorrectedInverseJacobian[0][0] = Flon * Flat[0][0] * InverseJacobian[0][0];
        CorrectedInverseJacobian[1][0] = Flon * Flat[1][1] * InverseJacobian[1][0];
        CorrectedInverseJacobian[2][0] = Flon * Flat[2][2] * InverseJacobian[2][0];
        CorrectedInverseJacobian[0][1] = Flon * Flat[0][0] * InverseJacobian[0][1];
        CorrectedInverseJacobian[1][1] = Flon * Flat[1][1] * InverseJacobian[1][1];
        CorrectedInverseJacobian[2][1] = Flon * Flat[2][2] * InverseJacobian[2][1];
        CorrectedInverseJacobian[0][2] = Flon * Flat[0][0] * InverseJacobian[0][2];
        CorrectedInverseJacobian[1][2] = Flon * Flat[1][1] * InverseJacobian[1][2];
        CorrectedInverseJacobian[2][2] = Flon * Flat[2][2] * InverseJacobian[2][2];
	
	std::cout << "Jc " << CorrectedInverseJacobian[0][0] << " 00 " << CorrectedInverseJacobian[1][0] << " 10 " <<  CorrectedInverseJacobian[2][0] << " 20 "  <<  CorrectedInverseJacobian[0][1] << " 01 "  <<  CorrectedInverseJacobian[1][1] << " 11 "  <<  CorrectedInverseJacobian[2][1] << " 21 " <<  CorrectedInverseJacobian[0][2] << " 02 " <<  CorrectedInverseJacobian[1][2] << " 12 " <<  CorrectedInverseJacobian[2][2] << " 22 "<< std::endl ;
    }

    void setSpeed(double x, double y, double theta) {
        double angular0 = InverseJacobian[0][0] * x + InverseJacobian[0][1] * y + InverseJacobian[0][2] * theta;
        double angular1 = InverseJacobian[1][0] * x + InverseJacobian[1][1] * y + InverseJacobian[1][2] * theta;
        double angular2 = InverseJacobian[2][0] * x + InverseJacobian[2][1] * y + InverseJacobian[2][2] * theta;

		/*double angular0 = CorrectedInverseJacobian[0][0] * x + CorrectedInverseJacobian[0][1] * y + CorrectedInverseJacobian[0][2] * theta;
        double angular1 = CorrectedInverseJacobian[1][0] * x + CorrectedInverseJacobian[1][1] * y + CorrectedInverseJacobian[1][2] * theta;
        double angular2 = CorrectedInverseJacobian[2][0] * x + CorrectedInverseJacobian[2][1] * y + CorrectedInverseJacobian[2][2] * theta;  *//*
        
            TODO_5: Edit the three lines above in order to use CorrectedInverseJacobian matrix
            instead of InverseJacobian matrix.
        */
        if(angular0/PI == 0 && angular1/PI == 0 && angular2/PI == 0){
            if(onlyOnce==0){
                printf("w0=%.2lf w1=%.2lf w2=%.2lf\n", angular0/PI, angular1/PI, angular2/PI);
                onlyOnce = 1;
            }
        }else{
            printf("w0=%.2lf w1=%.2lf w2=%.2lf\n", angular0/PI, angular1/PI, angular2/PI);
        }
        std_msgs::Float64 msg0;
            msg0.data = angular0 / PI;
        std_msgs::Float64 msg1;
            msg1.data = angular1 / PI;
        std_msgs::Float64 msg2;
            msg2.data = angular2 / PI;

        wheel_0.publish(msg0);
        wheel_1.publish(msg1);
        wheel_2.publish(msg2);

    }
};

void move_right_1m(double x,RobotinoOdometry* robotino, ros::Rate* rate, int c_thresh){
    int c = 0;
    while (ros::ok()) {
        if (c < c_thresh) {
            // move right 1 meter
            robotino->setSpeed(x, 0, 0);
        }else{
            break;
        }
        ros::spinOnce(); // process the ros communication
        rate->sleep();
        c++;
    }
};
void turn_90_degrees(RobotinoOdometry* robotino, ros::Rate* rate, int left, int c_thresh){
    // if left is 1, then the robot turns left, if not the robot
    // turns right
    double theta = 1.1;
    if(left != 1){
        theta = theta * -1;
    }
    int c = 0;
    while (ros::ok()) {
        if (c < c_thresh) {
            // turn 90 degrees
            robotino->setSpeed(0, 0, theta);
        }else{
            break;
        }
        ros::spinOnce(); // process the ros communication
        rate->sleep();
        c++;
    }
};

int main(int argc,char **argv)
{
    std::cout<<"Robotino Odometry Start..."<<std::endl;
    ros::init(argc,argv,"robotino_odometry");
    RobotinoOdometry robotino; // create ros node
    ros::Rate rate(20); // run 20 times in a second (the speed of the node)
    int c = 0;
    while (ros::ok()) {
        /*
            
 		TODO_1: Edit below to make robot go right by setting 
            x=0.5 (or another positive number for different speed), y=0 and theta=0 
            You should also edit if (c<133) part to set how long the movement commands
            will be sent.

            TODO_6: Edit below to make robot do double squares 
                (* go right 1 meter, turn 90 degrees,
                * go right 1 meter, turn 90 degrees, 
                * go right 1 meter, turn 90 degrees, 
                * go right 2 meters, turn -90 degrees, 
                * go right 1 meter, turn -90 degrees, 
                * go right 1 meter, turn -90 degrees,
                * go right 1 meter)
        */
        double x = 0.50199;
        double y = 0.0;
        double theta = 0.0;

		int c_thresh_right = 132;
		int c_thresh_turn = 88;
		move_right_1m(x,&robotino, &rate, c_thresh_right);
		turn_90_degrees(&robotino, &rate, 1, c_thresh_turn);
		move_right_1m(x,&robotino, &rate, c_thresh_right-6);
		turn_90_degrees(&robotino, &rate, 1, c_thresh_turn+1);
		move_right_1m(x,&robotino, &rate, c_thresh_right-6);
		turn_90_degrees(&robotino, &rate, 1, c_thresh_turn);
		move_right_1m(x,&robotino, &rate, c_thresh_right-6);
		move_right_1m(x,&robotino, &rate, c_thresh_right-6);
		turn_90_degrees(&robotino, &rate, -1, c_thresh_turn +1);
		move_right_1m(x,&robotino, &rate, c_thresh_right-5);
		turn_90_degrees(&robotino, &rate, -1, c_thresh_turn + 1);
		move_right_1m(x,&robotino, &rate, c_thresh_right-6);
		turn_90_degrees(&robotino, &rate, -1, c_thresh_turn + 2);
		move_right_1m(x,&robotino, &rate, c_thresh_right-6); 
		robotino.setSpeed(0.0f, 0.0f, 0.0f);
		break;
		/*
        if (c < 132) {            // sends movement commands to the robot
            robotino.setSpeed(x, y, theta);
        }else{
            // stops the robot
            robotino.setSpeed(0.0f, 0.0f, 0.0f);
            
            if(c==1000){
                break;
            }
        }*/
        ros::spinOnce(); // process the ros communication
        rate.sleep();
        c++;
    }
    robotino.setSpeed(0.0f, 0.0f, 0.0f);

    std::cout<<"Robotino Odometry Finish..." <<std::endl;
}
