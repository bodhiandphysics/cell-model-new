#include "givr.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cstdlib>
#include <iostream>



#include "givio.h"
#include "sim.hpp"
#include "types.hpp"
#include "cell.hpp"
#include "panel.h"

using namespace glm;
using namespace givr;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;

int main(void)
{
  namespace givio = giv::io; // perhaps better than giv::io
  givio::GLFWContext glContext;
  glContext.glMajorVesion(4)
      .glMinorVesion(0)
      .glForwardComaptability(true)
      .glCoreProfile()
      .glAntiAliasingSamples(4)
      .matchPrimaryMonitorVideoMode();

  std::cout << givio::glfwVersionString() << '\n';

  //
  // setup window (OpenGL context)
  //
  auto window =
      glContext.makeImGuiWindow(givio::Properties()
                                    .size(givio::dimensions{1000, 1000})
                                    .title("Curve surfing...")
                                    .glslVersionString("#version 330 core"));

  auto view = View(TurnTable(), Perspective());

  view.camera.translate(vec3(0,0,-5));

  auto linestyle = GL_Line(Width(2.), Colour(1.0, 1.0, 0.0));
  auto linestyle2 = GL_Line(Width(2.), Colour(1.0, 0.3, 0.3));
  auto linestyle3 = GL_Line(Width(2.), Colour(0.3, 1.0, 0.3));
  auto linestyle4 = GL_Line(Width(2.), Colour(0.6, 0.8, 1.0));
  auto boxstyle = GL_Line(Width(2.), Colour(1.0, 1.0, 1.0));

  float linalpha = .000000001f;
  float angalpha = .00000001f; 

  Sim sim = Sim(glm::vec2{-5.0f, 0}, 1.0f, 3, 3, linalpha, angalpha);

  float sub_step = 0.05;
  glClearColor(0.f, 0.f, 0.f, 0.f);
  mainloop(std::move(window), [&](float frame_time) {
    float frame_advance = 1.0;//frame_time;

    if (panel::play){ 
      for (float deltat = 0.f; deltat < frame_advance; deltat += sub_step)
        sim.sim_iteration(sub_step); //use gravity, don't use bounds

     } else {

      if (panel::single_step) {
        sim.sim_iteration(sub_step); //use gravity, don't use bounds
        panel::single_step = false;
      }
    }


    glClearColor(0.f, 0.f, 0.f, 0.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    auto wall = MultiLine();

    for (auto& cell: sim.cells)
      for (auto& awall: cell.walls) {

      auto p1 = awall.pos1;
      auto p2 = awall.pos2;
      wall.push_back(Line(Point1(p1->position.x, p1->position.y, 0),Point2(p2->position.x, p2->position.y, 0)));
      std::cout << "added a wall"; 


    }
   auto box = MultiLine();
   box.push_back(Line(Point1(vec2(-10.0f ,-10.0f),0),Point2(vec2(-10.0f ,10.0f),0)));
   box.push_back(Line(Point1(vec2(-10.0f ,-10.0f),0),Point2(vec2(10.0f ,-10.0f),0)));
   box.push_back(Line(Point1(vec2(10.0f ,10.0f),0),Point2(vec2(-10.0f ,10.0f),0)));
   box.push_back(Line(Point1(vec2(10.0f ,10.0f),0),Point2(vec2(10.0f ,-10.0f),0)));


    auto cellwall = createRenderable(wall, linestyle);
    draw(cellwall, view);
    auto boxrend = createRenderable(box,boxstyle);
    draw(boxrend,view);
    
  });
  exit(EXIT_SUCCESS);
}