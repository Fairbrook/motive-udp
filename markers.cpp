//======================================================================================================
// NaturalPoint Motive API Sample: Accessing Camera, Marker, and RigidBody Information
//======================================================================================================
#include <conio.h>
#include <thread>
#include <mutex>
#include <WinSock2.h>
#include <WS2tcpip.h>

#include <MotiveAPI.h>
#include "inputmanager.h"
#include "apilistener.h"
#include "transformmatrix.h"
#include "support.h"

#pragma comment(lib, "ws2_32.lib")
#define PORT     4020

using namespace MotiveAPI;
using namespace Support;
using namespace CameraVideoSupport;

int sockfd;
struct sockaddr_in servaddr;


// Test method
int ProcessFrame( int frameCounter )
{
    float   yaw, pitch, roll;
    float   x, y, z;
    float   qx, qy, qz, qw;
    bool    tracked;

    printf( "\rFrame #%d: %d Markers", frameCounter, MarkerCount() );
    printf( "\tAverage Marker Size: %f mm", MarkerAverageSize() * 1000 );

    for ( int i = 0; i < RigidBodyCount(); i++ )
    {
        bool rbTracked = RigidBodyTransform( i, &x, &y, &z, &qx, &qy, &qz, &qw, &yaw, &pitch, &roll );

        if ( rbTracked )
        {
            wchar_t name[256];

            RigidBodyName( i, name, 256 );
            wprintf( L"\n%s: Pos (%.3f, %.3f, %.3f) Orient (%.1f, %.1f, %.1f)\n", name,
                x, y, z, yaw, pitch, roll );
            // Send message to server
            float_t msg[10] = { x,y,z,qx,qy,qz,qw,yaw,pitch,roll };
            sendto(sockfd, (char *)msg, sizeof(float_t)*10, 0,
                (const struct sockaddr*)&servaddr, sizeof(servaddr));

            TransformMatrix xRot( TransformMatrix::RotateX( pitch * kRadToDeg ) );
            TransformMatrix yRot( TransformMatrix::RotateY( yaw * kRadToDeg ) );
            TransformMatrix zRot( TransformMatrix::RotateZ( roll * kRadToDeg ) );

            // Compose the local-to-world rotation matrix in XYZ (pitch, yaw, roll) order.

            TransformMatrix worldTransform = xRot * yRot * zRot;

            // Inject world-space coordinates of the origin.

            worldTransform.SetTranslation( x, y, z );

            // Invert the transform matrix to convert from a local-to-world to a world-to-local.

            worldTransform.Invert();

            printf( "\n>> Compare local rigid body coordinates with world-to-local converted markers\n" );

            float   mx, my, mz;
            float   tx, ty, tz;

            int markerCount = RigidBodyMarkerCount( i );
            for ( int j = 0; j < markerCount; ++j )
            {
                // Get the world-space coordinates of each rigid body marker.
                RigidBodyReconstructedMarker( i, j, tracked, mx, my, mz );

                // Get the rigid body's local coordinate for each marker.
                RigidBodyMarker( i, j, &tx, &ty, &tz );

                // Transform the rigid body point from world coordinates to local rigid body coordinates.
                // Any world-space point can be substituted here to transform it into the local space of
                // the rigid body.

                Point4  worldPnt( mx, my, mz, 1.0f );
                Point4  localPnt = worldTransform * worldPnt;

                printf( "\n  >> %d: Local: (%.3f, %.3f, %.3f) World-To-Local: (%.3f, %.3f, %.3f)\n", j + 1,
                    tx, ty, tz, localPnt[0], localPnt[1], localPnt[2] );
            }

            //== Invert the transform matrix so we can perform local-to-world.

            worldTransform.Invert();

            printf( "\n>> Compare world markers with local-to-world converted rigid body markers\n" );

            for ( int j = 0; j < markerCount; ++j )
            {
                // Get the world-space coordinates of each rigid body marker.
                RigidBodyReconstructedMarker( i, j, tracked, mx, my, mz );

                // Get the rigid body's local coordinate for each marker.
                RigidBodyMarker( i, j, &tx, &ty, &tz );

                // Transform the rigid body's local point to world coordinates.
                // Any local-space point can be substituted here to transform it into world coordinates.

                Point4  localPnt( tx, ty, tz, 1.0f );
                Point4  worldPnt = worldTransform * localPnt;

                printf( "\n  >> %d: World (%.3f, %.3f, %.3f) Local-To-World: (%.3f, %.3f, %.3f)\n", j + 1,
                    mx, my, mz, worldPnt[0], worldPnt[1], worldPnt[2] );
            }
            printf( "\n" );
        }
        else
        {
            wchar_t name[256];

            RigidBodyName( i, name, 256 );
            wprintf( L"\n\t%s: Not Tracked\n", name );
        }
    }

    // If calibrating, print out some state information.
    if ( frameCounter % 100 == 0 )
    {
        PrintCalibrationStateInfo();
    }

    return frameCounter;
}


// Main application
int main( int argc, char* argv[] )
{
    const wchar_t* calibrationFile = L"C:\\ProgramData\\OptiTrack\\Motive\\System Calibration.mcal";
    const wchar_t* profileFile = L"C:\\ProgramData\\OptiTrack\\Motive\\MotiveProfile.motive";

    if ( Initialize() != kApiResult_Success )
    {
        printf( "Unable to license Motive API, please make sure you have a hardware key, a device with a license built-in, or license in the license folder\n" );
        return 1;
    }

    // Attach listener for frame notifications
    APIListener listener;
    AttachListener( &listener );

    // Automatically Load a current camera calibration and profiles saved by Motive.
    int cameraCount = LoadCalibrationAndProfile( calibrationFile, profileFile );

    printf( "Initializing NaturalPoint Devices...\n\n" );

    WaitForCameraDiscovery( cameraCount );
    PrintConnectedCameras();
    PrintRigidBodies();

    FlushCameraQueues();

    InputManager inputManager;

    // hookup commands
    inputManager.RegisterKeyPressFunction( 'v', "Show Visible Object State",
        [] ()
        {
            printf( "\n*** Show Visible Object State ***\n" );
            PrintCameraHasVisibleMarkers();
        }
    );

    inputManager.RegisterKeyPressFunction( 'm', "Calibration - Auto-mask All Cameras",
        [] ()
        {
            printf( "\n*** Auto-mask All Cameras ***\n" );
            AutoMaskAllCameras();
        }
    );

    inputManager.RegisterKeyPressFunction( '5', "Calibration - Set Wand to CW-500",
        [] ()
        {
            printf( "\n*** Set Calibration Wand to the CW-500 ***\n" );
            SetCalibrationWand( eCalibrationWandType::WandLarge );
            PrintCalibrationWandSettings();
        }
    );
    inputManager.RegisterKeyPressFunction( '2', "Calibration - Set Wand to CW-250",
        [] ()
        {
            printf( "\n*** Set Calibration Wand to the CW-250 ***\n" );
            SetCalibrationWand( eCalibrationWandType::WandSmall );
            PrintCalibrationWandSettings();
        }
    );
    inputManager.RegisterKeyPressFunction( 'w', "Calibration - Start Wanding",
        [] ()
        {
            printf( "\n*** Begin wanding the volume ***\n" );
            StartCalibrationWanding();
        }
    );

    inputManager.RegisterKeyPressFunction( 'c', "Calibration - Start Calculation",
        [] ()
        {
            printf( "\n*** Start Calibration Calculation ***\n" );
            if ( !StartCalibrationCalculation() )
            {
                printf( "ERROR - Please wand the volume first by calling StartCalibrationWanding()\n" );
            }
        }
    );

    inputManager.RegisterKeyPressFunction( 'a', "Calibration - Apply Calculations",
        [] ()
        {
            printf( "\n*** Apply Calibration Calculations ***\n" );
            if ( ApplyCalibrationCalculation() )
            {
                printf( "SUCCESS - Calibration has been applied and calibration engine has been reset\n" );
            }
            else
            {
                printf( "ERROR - Please wand the volume first by calling StartCalibrationWanding()\n" );
            }
        }
    );

    // You could want to cancel the calibration after applying it if you don't want to set the ground plane (useful for tracking bars)
    inputManager.RegisterKeyPressFunction( 'x', "Calibration - Cancel",
        [] ()
        {
            printf( "\n*** Cancel Calibration ***\n" );
            CancelCalibration();
        }
    );

    inputManager.RegisterKeyPressFunction( 'd', "Calibration - Detect Calibration Square",
        [] ()
        {
            printf( "\n*** Calibration Square ***\n" );
            eCalibrationSquareType calibrationSquare = AutoDetectCalibrationSquare();
            wprintf( L"\t%d %s \n", calibrationSquare, MapToCalibrationSquareString( calibrationSquare ).c_str() );
        }
    );

    inputManager.RegisterKeyPressFunction( 'g', "Calibration - Set Ground Plane",
        [] ()
        {
            printf( "\n*** Set Ground Plane ***\n" );
            SetGroundPlane( false );
        }
    );

    inputManager.RegisterKeyPressFunction( 'G', "Calibration - Get Ground Plane Markers",
        [] ()
        {
            printf( "\n*** Ground PLane Markers ***\n" );
            std::vector<Core::cMarker> markers;
            if ( GetGroundPlaneMarkers( markers ) == kApiResult_Success )
            {
                int markeridx = 0;
                for ( Core::cMarker markerIdx : markers )
                {
                    PrintMarker( markeridx++ );
                }
            }
        }
    );

    inputManager.RegisterKeyPressFunction( 't', "Calibration - Translate Ground Plane",
        [] ()
        {
            printf( "\n*** Translate Ground Plane Markers ***\n" );
            TranslateGroundPlane( 1000.0f, 500.0f, -1000.0f );
        }
    );

    inputManager.RegisterKeyPressFunction( 'C', "Calibration - Check Current State",
        [] ()
        {
            printf( "\n*** Checking the Current State of Calibration ***\n" );
            eCalibrationState currentState = CalibrationState();
            wprintf( L"%s\n", MapToCalibrationStateString( currentState ).c_str() );

        }
    );

    //
    // Tracking Bar controls (Duo, Trio, Mini Duo 13)
    //
    inputManager.RegisterKeyPressFunction( 'u', "Calibration - Upload to Tracking Bar Camera (not recommended)",
        [] ()
        {

            const wchar_t* calibrationFile = L"C:\\ProgramData\\OptiTrack\\Motive\\System Calibration.mcal";

            printf( "\n*** Uploading Calibration to Tracking Bar's internal storage ***\n" );
            if ( LoadCalibration( calibrationFile ) == kApiResult_Success &&
                UploadCurrentCalibrationToTrackingBar() == kApiResult_Success )
            {
                printf( "SUCCESS - Uploaded calibration\n" );
            }
            else
            {
                printf( "ERROR - There was a problem uploading the calibration\n" );
            }
        }
    );

    //
    // Mini Duo 13 controls for 1kb reserved customer space on internal storage
    //
    inputManager.RegisterKeyPressFunction( 'e', "Erase/overwrite all 1kb of Mini Duo 13 reserved storage with 0's",
        [] ()
        {

            printf( "\n*** Erasing data on Mini Duo 13 reserved customer space ***\n" );

            // fill reserved storage (1kb) with 0's
            int bytesToErase = 128; // writes in 32 byte chunks only! will fill 0's to make 32 bytes for any remainder (see MotiveAPI.h notes)
            std::vector<uint8_t> writeBuffer;
            FillByteArray( writeBuffer, (uint8_t)0, bytesToErase );
            int bytesWritten = MiniDuo13SaveCustomerData( writeBuffer, bytesToErase );

            if ( bytesWritten == bytesToErase )
            {
                printf( "\nSuccess: requested %d to be erased, and erased %d bytes from device internal storage", bytesToErase, bytesWritten );
            }
            else
            {
                printf( "\nFailure: requested %d to be erased, and erased %d bytes from device internal storage", bytesToErase, bytesWritten );
            }

            printf( "\n" );
        }
    );
    inputManager.RegisterKeyPressFunction( 'f', "Fill/overwrite customer data on Mini Duo 13 reserved storage with 0xFF bytes",
        [] ()
        {
            printf( "\n*** Filling data on MiniDuo13 reserved customer space ***\n" );

            // write a series of 0xFF bytes to the device internal storage
            int bytesToFill = 64; // writes in 32 byte chunks only! will fill 0's to make 32 bytes for any remainder (see MotiveAPI.h notes)
            std::vector<uint8_t> writeBuffer;
            FillByteArray( writeBuffer, (uint8_t)0xFF, bytesToFill );
            int bytesWritten = MiniDuo13SaveCustomerData( writeBuffer, bytesToFill );

            if ( bytesWritten == bytesToFill )
            {
                printf( "\nSuccess: requested %d to be written, and wrote %d bytes from device internal storage", bytesToFill, bytesWritten );
            }
            else
            {
                printf( "\nFailure: requested %d to be written, and wrote %d bytes from device internal storage", bytesToFill, bytesWritten );
            }

            printf( "\n" );
        }
    );
    inputManager.RegisterKeyPressFunction( 'r', "Fill/overwrite customer data with random bytes on EEPROM",
        [] ()
        {
            printf( "\n*** Filling random data on MiniDuo13 reserved customer space ***\n" );

            // write a series of random bytes to the device internal storage 
            int bytesToFill = 32; // writes in 32 byte chunks only! will fill 0's to make 32 bytes for any remainder (see MotiveAPI.h notes)
            std::vector<uint8_t> writeBuffer;
            RandomFillByteArray( writeBuffer, bytesToFill );
            int bytesWritten = MiniDuo13SaveCustomerData( writeBuffer, bytesToFill );

            if ( bytesWritten == bytesToFill )
            {
                printf( "\nSuccess: requested %d to be written, and wrote %d bytes from device internal storage", bytesToFill, bytesWritten );
            }
            else
            {
                printf( "\nFailure: requested %d to be written, and wrote %d bytes from device internal storage", bytesToFill, bytesWritten );
            }

            printf( "\n" );
        }
    );
    inputManager.RegisterKeyPressFunction( 'p', "Print customer data from MiniDuo13 reserved customer space",
        [] ()
        {
            printf( "\n*** Printing MiniDuo13 customer data ***\n" );

            // read a series of bytes from the device internal storage
            int bytesToRead = 128; // can be any number 1-1024 (see MotiveAPI.h notes)
            std::vector<uint8_t> readBuffer;
            int bytesRead = MiniDuo13LoadCustomerData( readBuffer, bytesToRead );

            if ( bytesRead == bytesToRead )
            {
                printf( "\n" );
                for ( int i = 0; i < readBuffer.size(); i++ )
                {
                    printf( "%02X ", readBuffer[i] );

                    if ( (i + 1) % 16 == 0 )
                    {
                        printf( "\n" );
                    }
                }

                printf( "\nSuccess: requested %d to be read, and read %d bytes from device internal storage", bytesToRead, bytesRead );
            }
            else
            {
                printf( "\nFailure: requested %d to be read, and read %d bytes from device internal storage", bytesToRead, bytesRead );
            }

            printf( "\n" );
        }
    );
    inputManager.RegisterKeyPressFunction('k', "Remove all rigid bodies and create a rigid body with all available markers", []() {
        int num_markers = MarkerCount();
        if (num_markers < 3) printf("\nFailure: not enough markers: %d", num_markers);
        float positions[9];
        for (char m = 0; m < 3; m++) {
            float x, y, z;
            if (!MarkerXYZ(m, x, y, z)) {
                printf("\nFailure: could not retrieve %d marker", m);
                return;
            }
            positions[m * 3] = x;
            positions[m * 3+1] = y;
            positions[m * 3+2] = z;
        }
        ClearRigidBodies();
        const wchar_t name[6] = {'T','e', 'l','l', 'o', '\0'};
        CreateRigidBody(name, 0, 3, positions);
    });


    WSAData data;
    int wserr = WSAStartup(MAKEWORD(2, 2), &data);
    if (wserr != 0) {
        printf("Help\n");
        return 0;
    }

    // Create UDP socket
    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));

    // Fill server address info
    servaddr.sin_family = AF_INET;              // IPv4
    inet_pton(AF_INET, "169.254.6.200", &servaddr.sin_addr.s_addr);
    servaddr.sin_port = htons(PORT);          // Server port


    // set function to be called for each frame during ProcessFrames();
    inputManager.RegisterProcessFrameFunction( &ProcessFrame );
    inputManager.StartProcessingLoop( &listener );

    
  

    if ( inputManager.Save() )
    {
        CheckResult( SaveProfile( profileFile ) );
        CheckResult( SaveCalibration( calibrationFile ) );
    }

    inputManager.Shutdown();

    return 0;
}
