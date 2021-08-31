#include "geometrycentral/surface/flip_geodesics.h"
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

#include "args/args.hxx"
#include "imgui.h"

#include "split-mesh.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;

std::unique_ptr<ManifoldSurfaceMesh> mesh;
std::unique_ptr<VertexPositionGeometry> geometry;
polyscope::SurfaceMesh* psMesh;
std::vector<std::vector<SurfacePoint>> CURVES;
std::string MESHNAME;

/*
 * Read input curves on the surface.
 * Each curve is represented as a series of SurfacePoints.
 */
void readInputCurves(const std::string& filename) {

    std::ifstream curr_file(filename.c_str());
    std::string line;
    std::string X;   // re-written variable for holding the leading (indicator) char
    size_t idx;      // re-written variable for holding element index
    double tEdge;    // re-written variable for holding halfedge param
    int nCurves = 0; // the current # of curves read in

    if (curr_file.is_open()) {
        while (!curr_file.eof()) {
            getline(curr_file, line);
            std::istringstream iss(line);
            iss >> X;
            if (X == "v") {
                iss >> idx;
                CURVES[nCurves - 1].emplace_back(mesh->vertex(idx));
            } else if (X == "e") {
                iss >> idx >> tEdge;
                CURVES[nCurves - 1].emplace_back(mesh->edge(idx), tEdge);
            } else if (X == "n") {
                CURVES.emplace_back();
                nCurves += 1;
            }
        }
        curr_file.close();
    } else {
        std::cerr << "Could not open file <" << filename << ">" << std::endl;
    }
}

/*
 * Visualize the input curves.
 */
void displayCurves() {

    // Get the curve as a sequence of points in 3D space,
    std::vector<std::vector<Vector3>> path;
    for (size_t i = 0; i < CURVES.size(); i++) {
        std::vector<Vector3> curve;
        for (size_t j = 0; j < CURVES[i].size(); j++) {
            SurfacePoint pt = CURVES[i][j];
            curve.push_back(pt.interpolate(geometry->inputVertexPositions));
        }
        path.push_back(curve);
    }
    auto pathQ = psMesh->addSurfaceGraphQuantity("input curves", path);
    pathQ->setEnabled(true);
    pathQ->setColor({0, 0, 0});
    pathQ->setRadius(0.001);
}

/*
 * Display any Boundary Loops.
 */
void displayBoundary() {

    std::vector<Vector3> positions;
    std::vector<std::array<size_t, 2>> edgeInds;

    size_t idx = 0;
    for (BoundaryLoop bl : mesh->boundaryLoops()) {
        for (Halfedge he : bl.adjacentHalfedges()) {
            positions.emplace_back(geometry->inputVertexPositions[he.tailVertex()]);
            positions.emplace_back(geometry->inputVertexPositions[he.tipVertex()]);
            edgeInds.push_back({2 * idx, 2 * idx + 1});
            idx++;
        }
    }

    auto boundary = psMesh->addSurfaceGraphQuantity("boundary loops", positions, edgeInds);
    boundary->setEnabled(true);
}


void redraw() {
    polyscope::requestRedraw();
}


void functionCallback() {

    if (ImGui::Button("Split mesh and save")) {
        // writeSurfaceMesh(*mesh, *geometry, newMeshname);
        meshsplit::writeSplitEdgeOBJ(mesh.get(), geometry.get(), CURVES, MESHNAME);
    }
}


int main(int argc, char** argv) {

    // Configure the argument parser
    args::ArgumentParser parser("mesh-split");
    args::Positional<std::string> meshFilename(parser, "mesh", "A mesh file.");
    args::Positional<std::string> inputFilename(parser, "curve", "A curve file.");

    // Parse args
    try {
        parser.ParseCLI(argc, argv);
    } catch (args::Help) {
        std::cout << parser;
        return 0;
    } catch (args::ParseError e) {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return 1;
    }

    // If a mesh name was not given, use default mesh.
    std::string meshFilepath = "../../meshes/obj/bunny.obj";
    std::string inputFilepath = "../../sdf-heat-method/data/bunny-unsigned-constrained-curve.txt";
    if (meshFilename) {
        meshFilepath = args::get(meshFilename);
    }
    if (inputFilename) {
        inputFilepath = args::get(inputFilename);
    }

    // Load mesh
    std::tie(mesh, geometry) = readManifoldSurfaceMesh(meshFilepath);
    MESHNAME = polyscope::guessNiceNameFromPath(meshFilepath);

    geometry->requireVertexIndices();
    geometry->requireEdgeIndices();
    geometry->requireFaceIndices();

    // Initialize polyscope
    polyscope::init();

    // Set the callback function
    polyscope::state::userCallback = functionCallback;

    // Add mesh to GUI
    psMesh = polyscope::registerSurfaceMesh(MESHNAME, geometry->inputVertexPositions, mesh->getFaceVertexList(),
                                            polyscopePermutations(*mesh));

    // Load curves
    readInputCurves(inputFilepath);
    displayCurves();
    displayBoundary();

    // Give control to the polyscope gui
    polyscope::show();

    return EXIT_SUCCESS;
}