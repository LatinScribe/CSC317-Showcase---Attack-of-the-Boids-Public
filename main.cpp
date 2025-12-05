#include "boid.h"
#include <igl/get_seconds.h>
#include <igl/opengl/glfw/Viewer.h>
#include <GLFW/glfw3.h>
#include <Eigen/Sparse>
#include <algorithm>

// Some Global variables for the simulation
int num_boids = 100;
std::vector<Boid> boids;
Eigen::MatrixXd V_boid; // Vertices of a single boid mesh
Eigen::MatrixXi F_boid; // Faces of a single boid mesh

// Try playing with these weights!!!
double alignment_weight  = 1.0;
double cohesion_weight   = 1.0;
double separation_weight = 3.0;
double target_weight     = 4.5;

int main(int argc, char * argv[])
{
  // For now, using a simple tetrahedron (pyramid) as the boid mesh
  // the base is the "head" and the apex is the "tail"
  Eigen::MatrixXd V_tetra(4, 3);
  Eigen::MatrixXi F_tetra(4, 3);
  V_tetra << 0.0, 0.0, 1.0,
             1.0, 0.0, -1.0,
            -1.0, 0.0, -1.0,
             0.0, 1.0, -1.0;
  F_tetra << 0, 1, 2,
             0, 2, 3,
             0, 3, 1,
             1, 3, 2;

// A simple bird-like mesh for the boid
// based off the mesh from: https://medium.com/better-programming/mastering-flock-simulation-with-boids-c-opengl-and-imgui-5a3ddd9cb958
Eigen::MatrixXd V_bird_mesh(20, 3);
V_bird_mesh <<
  -0.06, -0.04, -0.20,
   0.06, -0.04, -0.20,
   0.06,  0.04, -0.20,
  -0.06,  0.04, -0.20,
  -0.04, -0.03,  0.20,
   0.04, -0.03,  0.20,
   0.04,  0.03,  0.20,
  -0.04,  0.03,  0.20,
   0.00,  0.00, -0.35,
   0.00,  0.00,  0.45,
   0.00,  0.05, -0.05,
   0.00, -0.02, -0.05,

   // right wing (corrected, outward)
   0.45,  0.02,  0.15,
   0.45,  0.02, -0.15,
   0.75,  0.00,  0.15,
   0.75,  0.00, -0.15,

   // left wing (corrected, outward)
  -0.45,  0.02,  0.15,
  -0.45,  0.02, -0.15,
  -0.75,  0.00,  0.15,
  -0.75,  0.00, -0.15;
  
Eigen::MatrixXi F_bird_mesh(34, 3);
F_bird_mesh <<
   8,  0,  1,
   8,  1,  2,
   8,  2,  3,
   8,  3,  0,
   0,  1,  5,
   0,  5,  4,
   1,  2,  6,
   1,  6,  5,
   3,  6,  2,
   3,  7,  6,
   0,  4,  7,
   0,  7,  3,
   4,  5,  6,
   4,  6,  7,
   4,  5,  9,
   5,  6,  9,
   6,  7,  9,
   7,  4,  9,
  10, 12, 13,
  11, 15, 14,
  10, 13, 15,
  10, 15, 11,
  10, 11, 14,
  10, 14, 12,
  13, 12, 14,
  13, 14, 15,
  10, 17, 16,
  11, 19, 18,
  10, 17, 19,
  10, 19, 11,
  10, 11, 18,
  10, 18, 16,
  17, 18, 16,
  17, 19, 18;

  bool use_bird_mesh = false;

  // Function to apply the current boid mesh based on the toggle
  auto apply_current_mesh = [&]()
  {
    if(use_bird_mesh)
    {
      V_boid = V_bird_mesh;
      F_boid = F_bird_mesh;
    }
    else
    {
      V_boid = V_tetra;
      F_boid = F_tetra;
    }
  };

  // Function to toggle between bird mesh and tetrahedron mesh
  auto toggle_boid_mesh = [&]()
  {
    use_bird_mesh = !use_bird_mesh;
    apply_current_mesh();
  };

  apply_current_mesh();

  // Initialize boids with random positions and velocities
  for(int i = 0; i < num_boids; i++)
  {
    Eigen::Vector3d pos = Eigen::Vector3d::Random() * 10.0;
    Eigen::Vector3d vel = Eigen::Vector3d::Random().normalized();
    boids.emplace_back(pos, vel);
  }

  // Setup libigl viewer (had some help for this from ChatGPT)
  // I removed the sparse matrix parts since they are not needed here
  // we don't do the sparse matrix animation steps
  igl::opengl::glfw::Viewer viewer;

  int current_num_boids = static_cast<int>(boids.size());

  // UI toggles for behaviors
  bool boids_enabled = true;
  bool alignment_enabled = true;
  bool cohesion_enabled = true;
  bool separation_enabled = true;
  bool target_enabled = true;

  auto print_options = [&]()
  {
    std::cout
      << "// Try playing with these weights as well in main.cpp (just remember to recompile)!!!\n"
      << "[1] Toggle boids (currently "      << (boids_enabled      ? "ON" : "OFF") << ")\n"
      << "[2] Toggle alignment (currently "  << (alignment_enabled  ? "ON" : "OFF") << ")\n"
      << "[3] Toggle cohesion (currently "   << (cohesion_enabled   ? "ON" : "OFF") << ")\n"
      << "[4] Toggle separation (currently " << (separation_enabled ? "ON" : "OFF") << ")\n"
      << "[5] Toggle target (currently "     << (target_enabled     ? "ON" : "OFF") << ")\n"
      << "[m, M] Toggle boid mesh (currently "  << (use_bird_mesh      ? "BIRD" : "TETRAHEDRON") << ")\n"
      << std::endl;
  };

  print_options();

  viewer.callback_key_down = [&](igl::opengl::glfw::Viewer &, unsigned int key, int)->bool {
    switch(key)
    {
      case GLFW_KEY_1: boids_enabled      = !boids_enabled;      break;
      case GLFW_KEY_2: alignment_enabled  = !alignment_enabled;  break;
      case GLFW_KEY_3: cohesion_enabled   = !cohesion_enabled;   break;
      case GLFW_KEY_4: separation_enabled = !separation_enabled; break;
      case GLFW_KEY_5: target_enabled     = !target_enabled;     break;
      case GLFW_KEY_M: toggle_boid_mesh();            break;
      case GLFW_KEY_M + GLFW_MOD_SHIFT: toggle_boid_mesh(); break;
      default: return false;
    }
    
    print_options();
    return true;
  };

  // Initially, target zero vector
  Boid::set_target(Eigen::Vector3d::Zero());

  // Mouse move callback to set target position
  viewer.callback_mouse_move = [&](igl::opengl::glfw::Viewer &, int x, int y)->bool
  {
    const Eigen::Vector4f viewport = viewer.core().viewport;
    if(viewport[2] <= 0.f || viewport[3] <= 0.f) return false;
    const double nx = (static_cast<double>(x) / static_cast<double>(viewport[2])) * 2.0 - 1.0;
    const double ny = (static_cast<double>(viewport[3] - y) / static_cast<double>(viewport[3])) * 2.0 - 1.0;
    Boid::set_target(Eigen::Vector3d(nx * 15.0, 0.0, ny * 15.0));
    return false;
  };

  // Mouse up callback to clear target position
  viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer & )->bool
  {
    const double current_time = igl::get_seconds();
    static double last_time = current_time;
    double dt = std::clamp(current_time - last_time, 1e-4, 0.033);
    last_time = current_time;

    // toggle weights based on enabled behaviors
    const double effective_alignment_weight  = alignment_enabled  ? alignment_weight  : 0.0;
    const double effective_cohesion_weight   = cohesion_enabled   ? cohesion_weight   : 0.0;
    const double effective_separation_weight = separation_enabled ? separation_weight : 0.0;
    const double effective_target_weight     = target_enabled     ? target_weight     : 0.0;

    const int active_count = boids_enabled ? std::min(current_num_boids, static_cast<int>(boids.size())) : 0;
    if(active_count <= 0)
    {
      viewer.data().clear();
      return false;
    }

    std::vector<Boid> snapshot(boids.begin(), boids.begin() + active_count);
    for(int i = 0; i < active_count; ++i)
    {
      boids[i].update(dt, snapshot, effective_alignment_weight, effective_cohesion_weight,
                      effective_separation_weight, effective_target_weight);

      for(int d = 0; d < 3; ++d)
      {
        // Wrap around boundaries
        if(boids[i].position[d] > 15.0)  boids[i].position[d] = -15.0;
        if(boids[i].position[d] < -15.0) boids[i].position[d] =  15.0;
      }
    }

    // Build combined mesh for all boids
    Eigen::MatrixXd V_all(active_count * V_boid.rows(), 3);
    Eigen::MatrixXi F_all(active_count * F_boid.rows(), 3);
    const Eigen::Vector3d scale(0.4, 0.4, 0.8);

    for(int i = 0; i < active_count; ++i)
    {
      // Compute orientation for each boid
      Eigen::Vector3d forward = boids[i].velocity;
      if(forward.norm() < 1e-3) forward = Eigen::Vector3d::UnitZ();
      forward.normalize();

      // Create orthonormal basis
      Eigen::Vector3d up = Eigen::Vector3d::UnitY();
      if(std::abs(forward.dot(up)) > 0.95) up = Eigen::Vector3d::UnitX();
      Eigen::Vector3d right = forward.cross(up).normalized();
      up = right.cross(forward).normalized();

      // Rotation matrix from boid local to world coordinates
      Eigen::Matrix3d R;
      R.col(0) = right;
      R.col(1) = up;
      R.col(2) = -forward;

      // Transform and place boid mesh
      for(int v = 0; v < V_boid.rows(); ++v)
      {
        Eigen::Vector3d local = V_boid.row(v).transpose();
        local = scale.asDiagonal() * local;
        Eigen::Vector3d world = R * local + boids[i].position;
        V_all.row(i * V_boid.rows() + v) = world.transpose();
      }

      F_all.block(i * F_boid.rows(), 0, F_boid.rows(), 3) =
          F_boid.array() + (i * V_boid.rows());
    }

    // Update viewer mesh
    viewer.data().clear();
    viewer.data().set_mesh(V_all, F_all);
    viewer.data().set_face_based(true);

    return false;
  };
  viewer.core().is_animating = true;
  viewer.launch_init(false);
  viewer.data().meshgl.init();

  // ============= Shading Stuff =================
  // cribed from the mass spring assignment
  // Essentially untouched
  // Ooof. All this to turn on double-sided lighting...
  igl::opengl::destroy_shader_program(viewer.data().meshgl.shader_mesh);
  {
    std::string mesh_vertex_shader_string =
R"(#version 150
  uniform mat4 view;
  uniform mat4 proj;
  uniform mat4 normal_matrix;
  in vec3 position;
  in vec3 normal;
  out vec3 position_eye;
  out vec3 normal_eye;
  in vec4 Ka;
  in vec4 Kd;
  in vec4 Ks;
  in vec2 texcoord;
  out vec2 texcoordi;
  out vec4 Kai;
  out vec4 Kdi;
  out vec4 Ksi;

  void main()
  {
    position_eye = vec3 (view * vec4 (position, 1.0));
    normal_eye = vec3 (normal_matrix * vec4 (normal, 0.0));
    normal_eye = normalize(normal_eye);
    gl_Position = proj * vec4 (position_eye, 1.0); //proj * view * vec4(position, 1.0);"
    Kai = Ka;
    Kdi = Kd;
    Ksi = Ks;
    texcoordi = texcoord;
  }
)";

    std::string mesh_fragment_shader_string =
R"(#version 150
  uniform mat4 view;
  uniform mat4 proj;
  uniform vec4 fixed_color;
  in vec3 position_eye;
  in vec3 normal_eye;
  uniform vec3 light_position_eye;
  vec3 Ls = vec3 (1, 1, 1);
  vec3 Ld = vec3 (1, 1, 1);
  vec3 La = vec3 (1, 1, 1);
  in vec4 Ksi;
  in vec4 Kdi;
  in vec4 Kai;
  in vec2 texcoordi;
  uniform sampler2D tex;
  uniform float specular_exponent;
  uniform float lighting_factor;
  uniform float texture_factor;
  out vec4 outColor;
  void main()
  {
    vec3 Ia = La * vec3(Kai);    // ambient intensity

    vec3 vector_to_light_eye = light_position_eye - position_eye;
    vec3 direction_to_light_eye = normalize (vector_to_light_eye);
    float dot_prod = dot (direction_to_light_eye, normalize(normal_eye));
    float clamped_dot_prod = abs(dot_prod);
    vec3 Id = Ld * vec3(Kdi) * clamped_dot_prod;    // Diffuse intensity

    vec3 reflection_eye = reflect (-direction_to_light_eye, normalize(normal_eye));
    vec3 surface_to_viewer_eye = normalize (-position_eye);
    float dot_prod_specular = dot (reflection_eye, surface_to_viewer_eye);
    dot_prod_specular = float(abs(dot_prod)==dot_prod) * max (dot_prod_specular, 0.0);
    float specular_factor = pow (dot_prod_specular, specular_exponent);
    vec3 Kfi = 0.5*vec3(Ksi);
    vec3 Lf = Ls;
    float fresnel_exponent = 2*specular_exponent;
    float fresnel_factor = 0;
    {
      float NE = max( 0., dot( normalize(normal_eye), surface_to_viewer_eye));
      fresnel_factor = pow (max(sqrt(1. - NE*NE),0.0), fresnel_exponent);
    }
    vec3 Is = Ls * vec3(Ksi) * specular_factor;    // specular intensity
    vec3 If = Lf * vec3(Kfi) * fresnel_factor;     // fresnel intensity
    vec4 color = vec4(lighting_factor * (If + Is + Id) + Ia + (1.0-lighting_factor) * vec3(Kdi),(Kai.a+Ksi.a+Kdi.a)/3);
    outColor = mix(vec4(1,1,1,1), texture(tex, texcoordi), texture_factor) * color;
    if (fixed_color != vec4(0.0)) outColor = fixed_color;
  }
)";

    igl::opengl::create_shader_program(
      mesh_vertex_shader_string,
      mesh_fragment_shader_string,
      {},
      viewer.data().meshgl.shader_mesh);
  }
  viewer.launch_rendering(true);
  viewer.launch_shut();
}
