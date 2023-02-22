

#include <mrpt/gui.h>
#include <mrpt/maps/CGasConcentrationGridMap2D.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/random.h>
#include <iostream>
#include <cstdlib>

using namespace std;

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace std;


struct MyConnectivityVisitor  : public mrpt::maps::CRandomFieldGridMap2D::ConnectivityDescriptor
{
    bool getEdgeInformation(
        const CRandomFieldGridMap2D* parent,
        size_t icx, size_t icy,
        size_t jcx, size_t jcy,
        double& out_edge_information) override
    {
        out_edge_information = 1.0 / (1.0 + icx + icy);
        return true;
    }
};

void RunGMRF(){
    const double X_SIZE = 15.0;
    const double Y_SIZE = 15.0;
    const double RESOLUTION = 0.5;

    mrpt::maps::CGasConcentrationGridMap2D gasmap(  CRandomFieldGridMap2D::mrGMRF_SD , 0, X_SIZE, 0, Y_SIZE,   RESOLUTION );
    auto conn =  mrpt::maps::CGasConcentrationGridMap2D::ConnectivityDescriptor::Ptr(  new MyConnectivityVisitor);
    gasmap.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    gasmap.setCellsConnectivity(conn);
    gasmap.clear(); 

    auto gl_data = mrpt::opengl::CPointCloud::Create();
    gl_data->setPointSize(3.0f);

    for (int i = 0; i < 20; i++) {
        const double value = getRandomGenerator().drawUniform(0.01, 0.99);
        const double x = getRandomGenerator().drawUniform(0.1, 0.95 * X_SIZE);
        const double y = getRandomGenerator().drawUniform(0.1, 0.95 * Y_SIZE);

        printf( "Observation: (x,y)=(%6.02f,%6.02f,)  => value: %6.03f\n", x, y,  value);
        gl_data->insertPoint(x, y, value);

        gasmap.insertIndividualReading( value, TPoint2D(x, y), false /*dont update map now*/);
    }

    // Update
    gasmap.updateMapEstimation();

    // 3D view:
    auto glObj = gasmap.getVisualization();

    mrpt::gui::CDisplayWindow3D win("Map", 840, 680);

    mrpt::opengl::COpenGLScene::Ptr& scene = win.get3DSceneAndLock();
    scene->insert(mrpt::opengl::stock_objects::CornerXYZSimple(1.0f, 4.0f));
    scene->insert(gl_data);
    scene->insert(glObj);
    win.unlockAccess3DScene();
    win.repaint();

    win.waitForKey();
}

int main(int argc, char** argv) {
     try
    {
        RunGMRF();
        return EXIT_SUCCESS;
    }
    catch (exception& e)
    {
        cout << "MRPT exception caught: " << e.what() << endl;
        return EXIT_FAILURE;
    }     
    return EXIT_SUCCESS;
}

