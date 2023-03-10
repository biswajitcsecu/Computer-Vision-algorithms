
#include <cstdlib>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/CTicTac.h>
#include <iostream>



using namespace std;

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace std;

void TestGeometry3D2()
{
    CPose3D iniPoseVehicle(  745.327749, 407.959716, 14.851070, -2.985091, 0.009412, 0.051315);
    CPoint3D GPSPosOnVehicle(-0.25, 0, 0.10);
    CPoint3D iniPoseGPS = iniPoseVehicle + GPSPosOnVehicle;
    printf( "Pose: %.6f,%.6f,%.6f", iniPoseGPS.x(), iniPoseGPS.y(), iniPoseGPS.z());
}

void TestGeometry3D()
{
    // The landmark (global) position: 3D (x,y,z)
    CPoint3D L(0, 4, 2);
    // Robot pose: 2D (x,y,phi)
    CPose2D R(2, 1, DEG2RAD(45.0f));
    // Camera pose relative to the robot: 6D (x,y,z,yaw,pitch,roll).
    CPose3D C(0.5f, 0.5f, 1.5f, DEG2RAD(-90.0f), 0.0_deg, DEG2RAD(-90.0f));
    cout << "L: " << L << endl;
    cout << "R: " << R << endl;
    cout << "C: " << C << endl;
    cout << "R+C:" << (R + C) << endl;
    // cout << (R+C).getHomogeneousMatrix();
    CPoint3D L2;
    CTicTac tictac;
    tictac.Tic();
    
    size_t i, N = 10000;
    
    for (i = 0; i < N; i++)
        L2 = L - (R + C);
    
    cout << "Computation in: " << 1e6 * tictac.Tac() / ((double)N) << " us" << endl;
    cout << "L': " << L2 << endl;

    // Reconstruct the landmark position:
    CPoint3D L3 = R + C + L2;
    cout << "R(+)C(+)L' = " << L3 << endl;
    cout << "Should be equal to L = " << L << endl;

    // Distance from the camera to the landmark
    cout << "|(R(+)C)-L|= " << (R + C).distanceTo(L) << endl;
    cout << "|L-(R(+)C)|= " << (R + C).distanceTo(L) << endl;
}


int main(int argc, char** argv) {
    try
    {
        TestGeometry3D();
        // TestGeometry3D2();
        return 0;
    }
    catch (exception& e)
    {
        std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
        return -1;
    }
    catch (...)
    {
        cerr << "Untyped excepcion!!";
        return -1;
    }
}

