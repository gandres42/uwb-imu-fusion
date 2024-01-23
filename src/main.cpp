#include <AHRS.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>

volatile sig_atomic_t sflag = 0;

void handle_sig(int sig)
{
    sflag = 1;
}


int main(int argc, char *argv[]) {
    std::cout << "Program Executing\n";
    signal(SIGINT, handle_sig);

    AHRS com = AHRS("/dev/ttyACM0");

    printf("Initializing\n\n");


    std::this_thread::sleep_for(std::chrono::milliseconds(1000));


    std::cout << "Pitch  |  Roll  |  Yaw  |  X-Accel  | Y-Accel  |  Z-Accel  |  Time  |" << std::endl;

    while( 1 == 1){
        std::cout << std::fixed << std::setprecision(2) << com.GetPitch() << "      " << com.GetRoll() << "   " << com.GetYaw() << "     " <<com.GetWorldLinearAccelX() << "     " << com.GetWorldLinearAccelY() << "       " << com.GetWorldLinearAccelZ() << "      " << com.GetLastSensorTimestamp() << "      " << '\r' << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if(sflag){
            sflag = 0;
            com.Close();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            break;
        }
    }
    printf("\nExit Caught... Closing device.\n");

    return 0;
}
