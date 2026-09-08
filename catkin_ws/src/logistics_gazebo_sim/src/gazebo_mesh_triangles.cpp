// Offline geometry audit using the same mesh loader as Gazebo Classic.
// Output is local metric triangle coordinates, NOT physics/contact validation.
#include <gazebo/common/Mesh.hh>
#include <gazebo/common/MeshManager.hh>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

int main(int argc, char **argv) {
  try {
    if (argc != 2) throw std::runtime_error("usage: gazebo_mesh_triangles MESH");
    const auto *mesh = gazebo::common::MeshManager::Instance()->Load(argv[1]);
    if (!mesh) throw std::runtime_error("Gazebo could not load mesh");
    std::ostringstream out;
    out << std::setprecision(17) << "[";
    bool first = true;
    for (unsigned s = 0; s < mesh->GetSubMeshCount(); ++s) {
      const auto *sub = mesh->GetSubMesh(s);
      if (sub->GetPrimitiveType() != gazebo::common::SubMesh::TRIANGLES ||
          sub->GetIndexCount() % 3 != 0)
        throw std::runtime_error("unsupported non-triangle mesh");
      for (unsigned i = 0; i < sub->GetIndexCount(); i += 3) {
        if (!first) out << ",";
        first = false;
        out << "[";
        for (unsigned j = 0; j < 3; ++j) {
          const auto index = sub->GetIndex(i + j);
          if (index >= sub->GetVertexCount()) throw std::runtime_error("bad index");
          const auto v = sub->Vertex(index);
          if (!std::isfinite(v.X()) || !std::isfinite(v.Y()) || !std::isfinite(v.Z()))
            throw std::runtime_error("nonfinite vertex");
          if (j) out << ",";
          out << "[" << v.X() << "," << v.Y() << "," << v.Z() << "]";
        }
        out << "]";
      }
    }
    if (first) throw std::runtime_error("empty triangle mesh");
    std::cout << out.str() << "]\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
