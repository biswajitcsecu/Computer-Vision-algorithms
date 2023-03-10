
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <alloca.h>
#include <vector>
#include "ospray/ospray_cpp.h"
#include "ospray/ospray_cpp/ext/rkcommon.h"
#include "rkcommon/utility/SaveImage.h"

using namespace rkcommon::math;

int main(int argc, const char **argv)
{
  // image size
  vec2i imgSize;
  imgSize.x = 1024; // width
  imgSize.y = 768; // height

  // camera
  vec3f cam_pos{0.f, 0.f, 0.f};
  vec3f cam_up{0.f, 1.f, 0.f};
  vec3f cam_view{0.1f, 0.f, 1.f};

  // triangle mesh data
  std::vector<vec3f> vertex = {vec3f(-1.0f, -1.0f, 3.0f),
      vec3f(-1.0f, 1.0f, 3.0f),
      vec3f(1.0f, -1.0f, 3.0f),
      vec3f(0.1f, 0.1f, 0.3f)};

  std::vector<vec4f> color = {vec4f(0.9f, 0.5f, 0.5f, 1.0f),
      vec4f(0.8f, 0.8f, 0.8f, 1.0f),
      vec4f(0.8f, 0.8f, 0.8f, 1.0f),
      vec4f(0.5f, 0.9f, 0.5f, 1.0f)};

  std::vector<vec3ui> index = {vec3ui(0, 1, 2), vec3ui(1, 2, 3)};



  // initialize OSPRay
  OSPError init_error = ospInit(&argc, argv);
  if (init_error != OSP_NO_ERROR)
    return init_error;

  {
    // create and setup camera
    ospray::cpp::Camera camera("perspective");
    camera.setParam("aspect", imgSize.x / (float)imgSize.y);
    camera.setParam("position", cam_pos);
    camera.setParam("direction", cam_view);
    camera.setParam("up", cam_up);
    camera.commit(); 

    // create and setup model and mesh
    ospray::cpp::Geometry mesh("mesh");
    mesh.setParam("vertex.position", ospray::cpp::CopiedData(vertex));
    mesh.setParam("vertex.color", ospray::cpp::CopiedData(color));
    mesh.setParam("index", ospray::cpp::CopiedData(index));
    mesh.commit();

    // put the mesh into a model
    ospray::cpp::GeometricModel model(mesh);
    model.commit();

    // put the model into a group 
    ospray::cpp::Group group;
    group.setParam("geometry", ospray::cpp::CopiedData(model));
    group.commit();

    // put the group into an instance 
    ospray::cpp::Instance instance(group);
    instance.commit();

    // put the instance in the world
    ospray::cpp::World world;
    world.setParam("instance", ospray::cpp::CopiedData(instance));

    // create and setup light for Ambient Occlusion
    ospray::cpp::Light light("ambient");
    light.commit();

    world.setParam("light", ospray::cpp::CopiedData(light));
    world.commit();

    // create renderer, choose Scientific Visualization renderer
    ospray::cpp::Renderer renderer("scivis");

    // complete setup of renderer
    renderer.setParam("aoSamples", 1);
    renderer.setParam("backgroundColor", 1.0f);
    renderer.commit();

    // create and setup framebuffer
    ospray::cpp::FrameBuffer framebuffer(  imgSize.x, imgSize.y, OSP_FB_SRGBA, OSP_FB_COLOR | OSP_FB_ACCUM);
    framebuffer.clear();

    // render one frame
    framebuffer.renderFrame(renderer, camera, world);

    // access framebuffer and write its content as PPM file
    uint32_t *fb = (uint32_t *)framebuffer.map(OSP_FB_COLOR);
    rkcommon::utility::writePPM("firstFrameCpp.ppm", imgSize.x, imgSize.y, fb);
    framebuffer.unmap(fb);
    std::cout << "rendering initial frame to firstFrameCpp.ppm" << std::endl;

    
    // converged image
    for (int frames = 0; frames < 10; frames++)
      framebuffer.renderFrame(renderer, camera, world);

    fb = (uint32_t *)framebuffer.map(OSP_FB_COLOR);
    rkcommon::utility::writePPM(  "accumulatedFrameCpp.ppm", imgSize.x, imgSize.y, fb);
    framebuffer.unmap(fb);
    std::cout << "rendering 10 accumulated frames to accumulatedFrameCpp.ppm"     << std::endl;

    ospray::cpp::PickResult res =   framebuffer.pick(renderer, camera, world, 0.5f, 0.5f);

    if (res.hasHit) {
      std::cout << "picked geometry [instance: " << res.instance.handle()
                << ", model: " << res.model.handle()       << ", primitive: " << res.primID << "]" << std::endl;
    }
  }
  ospShutdown();
  return 0;
}
