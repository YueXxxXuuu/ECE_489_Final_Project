#include <tistdtypes.h>
#include <coecsl.h>
#include "user_includes.h"
#include "math.h"


// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -25.47/180*PI;//-0.37;
float offset_Enc3_rad = 11.70/180*PI;//0.27;


// Your global varialbes.  

long mycount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;

#pragma DATA_SECTION(meaningless, ".my_vars")
float meaningless = 0.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];

#pragma DATA_SECTION(theta2array, ".my_arrs")
float theta2array[100];

long arrayindex = 0;

float p1 = 0.0300;
float p2 = 0.0128;
float p3 = 0.0076;
float p4 = 0.0753;
float p5 = 0.0298;

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

float g = 9.801;

float Theta1_old = 0;
float Omega1_old1 = 0;
float Omega1_old2 = 0;
float Omega1 = 0;

float Theta2_old = 0;
float Omega2_old1 = 0;
float Omega2_old2 = 0;
float Omega2 = 0;

float Theta3_old = 0;
float Omega3_old1 = 0;
float Omega3_old2 = 0;
float Omega3 = 0;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

//count variables
int count1 = 0;
int count2 = 0;

//LED status
int LED_status = 0;
int position = 0;

float t = 0;

float minimum_velocity1 = 0.1;
float minimum_velocity2 = 0.1;
float minimum_velocity3 = 0.1;

float ufric1 = 0.0;
float ufric2 = 0.0;
float ufric3 = 0.0;
float lambda = 1;

float Viscous_positive1 = 0.1900;
float Viscous_positive2 = 0.2500;
float Viscous_positive3 = 0.1500;

float Coulomb_positive1 = 0.3790;
float Coulomb_positive2 = 0.4759;
float Coulomb_positive3 = 0.5339;

float Viscous_negative1 = 0.1900;
float Viscous_negative2 = 0.2500;
float Viscous_negative3 = 0.2132;

float Coulomb_negative1 = -0.3800;
float Coulomb_negative2 = -0.5031;
float Coulomb_negative3 = -0.5190;

float slope_between_miminums1 = 3.6;
float slope_between_miminums2 = 3.6;
float slope_between_miminums3 = 3.6;


float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;
float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;
float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;
float thetaz = 0;
float thetax = 0;
float thetay = 0;
float R11 = 0;
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;
float RT11 = 0;
float RT12 = 0;
float RT13 = 0;
float RT21 = 0;
float RT22 = 0;
float RT23 = 0;
float RT31 = 0;
float RT32 = 0;
float RT33 = 0;

float x = 0.0;
float x_old = 0.0;
float dx = 0.0;
float dx_old1 = 0.0;
float dx_old2 = 0.0;

float y = 0.0;
float y_old = 0.0;
float dy = 0.0;
float dy_old1 = 0.0;
float dy_old2 = 0.0;

float z = 0.0;
float z_old = 0.0;
float dz = 0.0;
float dz_old1 = 0.0;
float dz_old2 = 0.0;

float x_desired = 0.0;
float dx_desired = 0.0;
float y_desired = 0.0;
float dy_desired = 0.0;
float z_desired = 0.0;
float dz_desired = 0.0;

float Kpx = 0.5;
float Kdx = 0.025;
float Kpy = 0.5;
float Kdy = 0.025;
float Kpz = 0.5;
float Kdz = 0.025;

float tau1_temp = 0;
float tau2_temp = 0;
float tau3_temp = 0;

float JTR11 = 0.0;
float JTR12 = 0.0;
float JTR13 = 0.0;
float JTR21 = 0.0;
float JTR22 = 0.0;
float JTR23 = 0.0;
float JTR31 = 0.0;
float JTR32 = 0.0;
float JTR33 = 0.0;

float temp1 = 0.0;
float temp2 = 0.0;
float temp3 = 0.0;

int idx;

float x_prev;
float y_prev;
float z_prev;

float x_next;
float y_next;
float z_next;

float t_delta;

struct topoint_type{
    float x;
    float y;
    float z;
    int stiff;
    float thetaZ;
};

struct topoint_type point[14] = {
                           {5.492,     0,   17, 0, 0},
                           {1.175, 13.78, 8.22, 0, 0},
                           {1.154, 13.90,  5.1, 1, 0},
                           {1.175, 13.78, 8.22, 1, 0},
                           {10.58,  3.89, 8.49, 0, 0},
                           {14.61,  4.22, 8.30, 0, 0},
                           {16.25,  1.51, 8.30, 2, -0.927293432},
                           {11.88,  2.39, 8.30, 2, 2.87979326579},
                           {15.30, -2.43, 8.30, 2, -0.927293432},
                           {14.89, -1.196, 15.3, 0, 0},
                           {14.35, -5.5, 14.6, 0, 0},
                           {14.35, -5.5, 13.55, 3, 0},
                           {14.35, -5.5, 13.55, 3, 0},
                           {   10,     0,   20, 0, 0}
                          };

// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {

    Omega1 = (theta1motor - Theta1_old)/0.001;
    Omega1 = (Omega1 + Omega1_old1 + Omega1_old2)/3.0;
    Theta1_old = theta1motor;
    Omega1_old2 = Omega1_old1;
    Omega1_old1 = Omega1;

    Omega2 = (theta2motor - Theta2_old)/0.001;
    Omega2 = (Omega2 + Omega2_old1 + Omega2_old2)/3.0;
    Theta2_old = theta2motor;
    Omega2_old2 = Omega2_old1;
    Omega2_old1 = Omega2;

    Omega3 = (theta3motor - Theta3_old)/0.001;
    Omega3 = (Omega3 + Omega3_old1 + Omega3_old2)/3.0;
    Theta3_old = theta3motor;
    Omega3_old2 = Omega3_old1;
    Omega3_old1 = Omega3;

    // Jacobian Transpose
    cosq1 = cos(theta1motor);
    sinq1 = sin(theta1motor);
    cosq2 = cos(theta2motor);
    sinq2 = sin(theta2motor);
    cosq3 = cos(theta3motor);
    sinq3 = sin(theta3motor);
    JT_11 = -10*sinq1*(cosq3 + sinq2);
    JT_12 = 10*cosq1*(cosq3 + sinq2);
    JT_13 = 0;
    JT_21 = 10*cosq1*(cosq2 - sinq3);
    JT_22 = 10*sinq1*(cosq2 - sinq3);
    JT_23 = -10*(cosq3 + sinq2);
    JT_31 = -10*cosq1*sinq3;
    JT_32 = -10*sinq1*sinq3;
    JT_33 = -10*cosq3;


    // Rotation zxy and its Transpose
    cosz = cos(thetaz);
    sinz = sin(thetaz);
    cosx = cos(thetax);
    sinx = sin(thetax);
    cosy = cos(thetay);
    siny = sin(thetay);
    RT11 = R11 = cosz*cosy-sinz*sinx*siny;
    RT21 = R12 = -sinz*cosx;
    RT31 = R13 = cosz*siny+sinz*sinx*cosy;
    RT12 = R21 = sinz*cosy+cosz*sinx*siny;
    RT22 = R22 = cosz*cosx;
    RT32 = R23 = sinz*siny-cosz*sinx*cosy;
    RT13 = R31 = -cosx*siny;
    RT23 = R32 = sinx;
    RT33 = R33 = cosx*cosy;

    x = 10*cosq1*(cosq3+sinq2);
    y = 10*sinq1*(cosq3+sinq2);
    z = 10*(1+cosq2-sinq3);

    dx = (x - x_old)/0.001;
    dx = (dx + dx_old1 + dx_old2)/3.0;
    x_old = x;
    dx_old2 = dx_old1;
    dx_old1 = dx;

    dy = (y - y_old)/0.001;
    dy = (dy + dy_old1 + dy_old2)/3.0;
    y_old = y;
    dy_old2 = dy_old1;
    dy_old1 = dy;

    dz = (z - z_old)/0.001;
    dz = (dz + dz_old1 + dz_old2)/3.0;
    z_old = z;
    dz_old2 = dz_old1;
    dz_old1 = dz;

    mycount = mycount%14000;
    t = mycount/1000.0;


    idx = ceil(t);
    if(idx == 14)
        idx = 0;

    if(idx == 0){
        t_delta = t - 13;
        x_prev = point[13].x;
        x_next = point[idx].x;
        x_desired = t_delta*(x_next-x_prev) + x_prev;

        y_prev = point[13].y;
        y_next = point[idx].y;
        y_desired = t_delta*(y_next-y_prev) + y_prev;

        z_prev = point[13].z;
        z_next = point[idx].z;
        z_desired = t_delta*(z_next-z_prev) + z_prev;

        Kpx = 0.5;
        Kdx = 0.025;
        Kpy = 0.5;
        Kdy = 0.025;
        Kpz = 0.5;
        Kdz = 0.025;

        thetaz = point[idx].thetaZ;

    }
    else{
        t_delta = t - (idx-1);
        x_prev = point[idx - 1].x;
        x_next = point[idx].x;
        x_desired = t_delta*(x_next-x_prev) + x_prev;

        y_prev = point[idx - 1].y;
        y_next = point[idx].y;
        y_desired = t_delta*(y_next-y_prev) + y_prev;

        z_prev = point[idx - 1].z;
        z_next = point[idx].z;
        z_desired = t_delta*(z_next-z_prev) + z_prev;

        if(point[idx].stiff == 3){
            Kpx = 0.5;
            Kdx = 0.025;
            Kpy = 0.5;
            Kdy = 0.025;
            Kpz = 0.1;
            Kdz = 0;
        }
        else if(point[idx].stiff == 2){
            Kpx = 0.5;
            Kdx = 0.025;
            Kpy = 0.1;
            Kdy = 0;
            Kpz = 0.5;
            Kdz = 0.025;
        }
        else if(point[idx].stiff == 1){
            Kpx = 0.1;
            Kdx = 0;
            Kpy = 0.1;
            Kdy = 0;
            Kpz = 0.5;
            Kdz = 0.025;
        }
        else{
            Kpx = 0.5;
            Kdx = 0.025;
            Kpy = 0.5;
            Kdy = 0.025;
            Kpz = 0.5;
            Kdz = 0.025;
        }

        thetaz = point[idx].thetaZ;
    }

    if(Omega1 > minimum_velocity1){
        ufric1 = Viscous_positive1*Omega1+Coulomb_positive1;
    }
    else if(Omega1 < -minimum_velocity1){
        ufric1 = Viscous_negative1*Omega1+Coulomb_negative1;
    }
    else{
        ufric1 = slope_between_miminums1*Omega1;
    }

    if(Omega2 > minimum_velocity2){
        ufric2 = Viscous_positive2*Omega2+Coulomb_positive2;
    }
    else if(Omega2 < -minimum_velocity2){
        ufric2 = Viscous_negative2*Omega2+Coulomb_negative2;
    }
    else{
        ufric2 = slope_between_miminums2*Omega2;
    }

    if(Omega3 > minimum_velocity3){
        ufric3 = Viscous_positive3*Omega3+Coulomb_positive3;
    }
    else if(Omega3 < -minimum_velocity3){
        ufric3 = Viscous_negative3*Omega3+Coulomb_negative3;
    }
    else{
        ufric3 = slope_between_miminums3*Omega3;
    }

    JTR11 = JT_11*R11+JT_12*R21+JT_13*R31;
    JTR12 = JT_11*R12+JT_12*R22+JT_13*R32;
    JTR13 = JT_11*R13+JT_12*R23+JT_13*R33;
    JTR21 = JT_21*R11+JT_22*R21+JT_23*R31;
    JTR22 = JT_21*R12+JT_22*R22+JT_23*R32;
    JTR23 = JT_21*R13+JT_22*R23+JT_23*R33;
    JTR31 = JT_31*R11+JT_32*R21+JT_33*R31;
    JTR32 = JT_31*R12+JT_32*R22+JT_33*R32;
    JTR33 = JT_31*R13+JT_32*R23+JT_33*R33;

    temp1 = Kpx*RT11*(x_desired - x)+Kpx*RT12*(y_desired - y)+Kpx*RT13*(z_desired - z)+Kdx*RT11*(dx_desired - dx)+Kdx*RT12*(dy_desired - dy)+Kdx*RT13*(dz_desired - dz);
    temp2 = Kpy*RT21*(x_desired - x)+Kpy*RT22*(y_desired - y)+Kpy*RT23*(z_desired - z)+Kdy*RT21*(dx_desired - dx)+Kdy*RT22*(dy_desired - dy)+Kdy*RT23*(dz_desired - dz);
    temp3 = Kpz*RT31*(x_desired - x)+Kpz*RT32*(y_desired - y)+Kpz*RT33*(z_desired - z)+Kdz*RT31*(dx_desired - dx)+Kdz*RT32*(dy_desired - dy)+Kdz*RT33*(dz_desired - dz);

    tau1_temp = JTR11*temp1 + JTR12*temp2 + JTR13*temp3;
    if(tau1_temp > 5)
        tau1_temp = 5;
    if(tau1_temp < -5)
        tau1_temp = -5;

    tau2_temp = JTR21*temp1 + JTR22*temp2 + JTR23*temp3;
    if(tau2_temp > 5)
        tau2_temp = 5;
    if(tau2_temp < -5)
        tau2_temp = -5;

    tau3_temp = JTR31*temp1 + JTR32*temp2 + JTR33*temp3;
    if(tau3_temp > 5)
        tau3_temp = 5;
    if(tau3_temp < -5)
        tau3_temp = -5;

    *tau1 = tau1_temp;
    *tau2 = tau2_temp;
    *tau3 = tau3_temp;

    //Motor torque limitation(Max: 5 Min: -5)

    // save past states
    if ((mycount%50)==0) {

        theta1array[arrayindex] = theta1motor;
        theta2array[arrayindex] = theta2motor;

        if (arrayindex >= 100) {
            arrayindex = 0;
        } else {
            arrayindex++;
        }

    }

    if ((mycount%500)==0) {
        if (whattoprint > 0.5) {
            serial_printf(&SerialA, "I love robotics\n\r");
        } else {
            printtheta1motor = theta1motor;
            printtheta2motor = theta2motor;
            printtheta3motor = theta3motor;
            SWI_post(&SWI_printf); //Using a SWI to fix SPI issue from sending too many floats.
        }
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
        // GpioDataRegs.GPBSET.bit.GPIO60 = 1
        // GpioDataRegs.GPBCLR.bit.GPIO60 = 1
        // GpioDataRegs.GPBDAT.bit.GPIO60 = 1 sets
        //                               = 0 clears
    }


    if((count1*(count2%5))%500 == 0){
        GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
    }

    Simulink_PlotVar1 = x;
    Simulink_PlotVar2 = x_desired;
    Simulink_PlotVar3 = z;
    Simulink_PlotVar4 = z_desired;

    if(count1%500 ==0){
        count2 ++;
    }
    count1 ++;
    mycount++;
}

void printing(void){
    serial_printf(&SerialA, "%.2f %.2f,%.2f   \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI);
}

