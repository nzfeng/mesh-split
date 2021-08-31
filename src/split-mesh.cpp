#include "split-mesh.h"

namespace meshsplit {


/*
 * Helper function for writeSplitEdgeOBJ(). Duplicate all vertices of the input curves, except for the endpoints.
 */
void duplicateVertices(ManifoldSurfaceMesh* mesh, VertexPositionGeometry* geometry,
                       const std::vector<std::vector<SurfacePoint>>& curves, std::vector<Vector3>& dualVertices,
                       size_t* idx_map) {

    size_t idx = 0;
    for (size_t k = 0; k < curves.size(); k++) {
        for (size_t i = 1; i < curves[k].size() - 1; i++) {
            dualVertices.push_back(geometry->inputVertexPositions[curves[k][i].vertex]);
            idx_map[geometry->vertexIndices[curves[k][i].vertex]] = mesh->nVertices() + idx;
            idx++;
        }
    }
}

/*
 * Helper function for writeSplitEdgeOBJ(). After interior vertices have been duplicated, edit faces adjacent to the
 * curve s.t. they contain the right vertices.
 */
void updateFaces(ManifoldSurfaceMesh* mesh, VertexPositionGeometry* geometry,
                 const std::vector<std::vector<SurfacePoint>>& curves, Eigen::MatrixXi& faces, const size_t* idx_map) {

    // Use halfedge to traverse edges adjacent to each interior vertex.
    for (size_t k = 0; k < curves.size(); k++) {
        for (size_t i = 1; i < curves[k].size() - 1; i++) {
            Vertex v = curves[k][i].vertex;
            size_t idx = geometry->vertexIndices[v];

            // Determine the "starting" halfedge that we're going to go CCW from
            Halfedge he_curr;
            bool he_found = false;

            // Assert that the vertex we seek is indeed among the adjacent vertices.
            // If this fails, he_curr won't be set in the codeblock below, leading
            // to a segfault when we try to access members of he_curr
            bool flag = false;
            for (Vertex vert : v.adjacentVertices()) {
                if (geometry->vertexIndices[vert] == geometry->vertexIndices[curves[k][i + 1].vertex]) {
                    flag = true;
                }
            }
            assert(flag);

            // sometimes this loop ran forever when I had duplicate vertices in the OBJ file...
            for (Halfedge he : v.outgoingHalfedges()) {
                if (geometry->vertexIndices[he.tipVertex()] == geometry->vertexIndices[curves[k][i + 1].vertex]) {
                    he_found = true;
                    he_curr = he;
                    break;
                }
            }
            assert(he_found);
            assert(he_curr.vertex() == v);

            // Go CCW from this starting halfedge until we hit the previous vertex, meaning we check a "fan" on one side
            // of the curve.
            while (geometry->vertexIndices[he_curr.tipVertex()] != geometry->vertexIndices[curves[k][i - 1].vertex]) {
                size_t face_idx = geometry->faceIndices[he_curr.face()];
                assert(face_idx < mesh->nFaces());

                if (faces(face_idx, 0) == idx) {
                    faces(face_idx, 3) = idx_map[idx];
                    faces(face_idx, 6) = 1;
                } else if (faces(face_idx, 1) == idx) {
                    faces(face_idx, 4) = idx_map[idx];
                    faces(face_idx, 7) = 1;
                } else if (faces(face_idx, 2) == idx) {
                    faces(face_idx, 5) = idx_map[idx];
                    faces(face_idx, 8) = 1;
                } else {
                    std::cerr << "face doesn't have matching index: "
                              << "<" << idx << "> " << faces(face_idx, 0) << " " << faces(face_idx, 1) << " "
                              << faces(face_idx, 2) << std::endl;
                }
                // get the next halfedge in the "fan"
                he_curr = he_curr.next().next().twin();
            }
        }
    }

    // Finally, make changes permanent.
    for (size_t i = 0; i < mesh->nFaces(); i++) {
        if (faces(i, 6)) {
            faces(i, 0) = faces(i, 3);
        }
        if (faces(i, 7)) {
            faces(i, 1) = faces(i, 4);
        }
        if (faces(i, 8)) {
            faces(i, 2) = faces(i, 5);
        }
    }
}
/*
 * Take in a mesh and its geometry, and a set of curves assumed to be constrained to mesh edges.
 * Then write an updated OBJ file. Ideally, I would split the mesh along the input curves and update the mesh
 * accordingly, before using Geometry Central's built-in mesh I/O functions, but I'm too lazy to figure out how to
 * handle element construction/pointer stuff in Geometry Central right now.
 */
void writeSplitEdgeOBJ(ManifoldSurfaceMesh* mesh, VertexPositionGeometry* geometry,
                       const std::vector<std::vector<SurfacePoint>>& curves, const std::string& meshname) {

    // Duplicate interior vertices.
    std::vector<Vector3> dualVertices;
    size_t idx_map[mesh->nVertices()];
    assert(sizeof(idx_map) / sizeof(idx_map[0]) == mesh->nVertices());
    duplicateVertices(mesh, geometry, curves, dualVertices, idx_map);
    std::cerr << "interior vertices duplicated" << std::endl;

    // Update vertex indices in faces.
    Eigen::MatrixXi faces(
        mesh->nFaces(),
        9); // first 3 values are old indices; next 3 are updated indices; next 3 are whether vertex has been set
    for (Face f : mesh->faces()) {
        size_t i = geometry->faceIndices[f];
        int cpt = 0;
        for (Vertex v : f.adjacentVertices()) {
            faces(i, cpt) = geometry->vertexIndices[v];
            cpt++;
        }

        faces(i, 6) = 0;
        faces(i, 7) = 0;
        faces(i, 8) = 0;
    }
    updateFaces(mesh, geometry, curves, faces, idx_map);
    std::cerr << "faces updated" << std::endl;

    std::ostringstream stringStream;
    stringStream << "../" << meshname << "_split.obj";
    std::string filename = stringStream.str();
    FILE* f;
    f = fopen(filename.c_str(), "wb");
    if (f == NULL) {
        printf("Error %d \n", errno);
    }

    // Write vertices.
    for (size_t i = 0; i < mesh->nVertices(); i++) {
        Vector3 v = geometry->inputVertexPositions[i];
        fprintf(f, "v %f %f %f \n", v[0], v[1], v[2]);
    }
    std::cerr << "vertices written" << std::endl;
    for (size_t i = 0; i < dualVertices.size(); i++) {
        fprintf(f, "v %f %f %f \n", dualVertices[i][0], dualVertices[i][1], dualVertices[i][2]);
    }
    std::cerr << "dual vertices written" << std::endl;

    // Write faces.
    for (size_t i = 0; i < mesh->nFaces(); i++) {
        fprintf(f, "f %d %d %d\n", faces(i, 0) + 1, faces(i, 1) + 1, faces(i, 2) + 1); // .obj file is 1-indexed!
    }
    std::cerr << "faces written" << std::endl;

    // Write curves
    for (size_t k = 0; k < curves.size(); k++) {
        fprintf(f, "l");
        for (size_t i = 1; i < curves[k].size() - 1; i++) {
            fprintf(f, " %zu %zu", geometry->vertexIndices[curves[k][i].vertex],
                    idx_map[geometry->vertexIndices[curves[k][i].vertex]]); // keep 0-indexing
        }
        fprintf(f, "\n");
        fprintf(f, "e %zu %zu\n", geometry->vertexIndices[curves[k][0].vertex],
                geometry->vertexIndices[curves[k][curves[k].size() - 1].vertex]); // keep 0-indexing
    }
    fclose(f);
}


} // namespace meshsplit