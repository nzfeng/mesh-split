#pragma once

#include "geometrycentral/surface/flip_geodesics.h"
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;

namespace meshsplit {

void duplicateVertices(ManifoldSurfaceMesh* mesh, VertexPositionGeometry* geometry,
                       const std::vector<std::vector<SurfacePoint>>& curves, std::vector<Vector3>& dualVertices,
                       size_t* idx_map);

void updateFaces(ManifoldSurfaceMesh* mesh, VertexPositionGeometry* geometry,
                 const std::vector<std::vector<SurfacePoint>>& curves, Eigen::MatrixXi& faces, const size_t* idx_map);

void writeSplitEdgeOBJ(ManifoldSurfaceMesh* mesh, VertexPositionGeometry* geometry,
                       const std::vector<std::vector<SurfacePoint>>& curves, const std::string& meshname);

} // namespace meshsplit