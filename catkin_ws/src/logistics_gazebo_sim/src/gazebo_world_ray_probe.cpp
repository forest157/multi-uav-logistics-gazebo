// Read x0 y0 z0 x1 y1 z1 lines from stdin; query the loaded physics engine.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <iomanip>
#include <iostream>
#include <stdexcept>

int main(int argc, char **argv) {
  try {
    if (argc != 2) throw std::runtime_error("usage: gazebo_world_ray_probe WORLD");
    if (!gazebo::setupServer()) throw std::runtime_error("server setup failed");
    const auto world = gazebo::loadWorld(argv[1]);
    if (!world) throw std::runtime_error("world load failed");
    world->Step(1);
    const auto ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        world->Physics()->CreateShape("ray", gazebo::physics::CollisionPtr()));
    if (!ray) throw std::runtime_error("physics engine has no ray shape");
    double x0, y0, z0, x1, y1, z1;
    std::cout << std::setprecision(17);
    while (std::cin >> x0 >> y0 >> z0 >> x1 >> y1 >> z1) {
      ray->SetPoints({x0,y0,z0}, {x1,y1,z1});
      double distance = 0;
      std::string entity;
      ray->GetIntersection(distance, entity);
      // Names in the pinned asset contain no quoting characters.
      if (entity.find_first_of("\"\\\n\r") != std::string::npos)
        throw std::runtime_error("unsupported entity name");
      std::cout << "{\"distance\":" << distance << ",\"entity\":\""
                << entity << "\"}\n";
    }
    if (!std::cin.eof()) throw std::runtime_error("invalid ray input");
    gazebo::shutdown();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << "\n";
    gazebo::shutdown();
    return 1;
  }
}
