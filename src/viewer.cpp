#include "solar/viewer.hpp"

#include <GL/glew.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <future>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace solar {
namespace {

constexpr float kPanelWidth = 370.0f;

constexpr const char* kVertexShader = R"glsl(
#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aColor;
uniform mat4 uViewProjection;
out vec3 vWorld;
out vec3 vColor;
void main() {
  vWorld = aPosition;
  vColor = aColor;
  gl_Position = uViewProjection * vec4(aPosition, 1.0);
}
)glsl";

constexpr const char* kFragmentShader = R"glsl(
#version 330 core
in vec3 vWorld;
in vec3 vColor;
uniform vec3 uSunDirection;
uniform bool uUnlit;
uniform bool uCellPass;
uniform bool uShellHeat;
uniform samplerBuffer uShellIrradiance;
out vec4 fragColor;

vec3 irradianceColor(float irradiance) {
  float t = clamp(irradiance / 1000.0, 0.0, 1.0);
  if (t < 0.25) return vec3(0.04, 0.22 + 1.6 * t, 0.75 + t);
  if (t < 0.5) return vec3(0.04, 0.62 + 0.9 * (t - 0.25),
                           1.0 - 2.5 * (t - 0.25));
  if (t < 0.75) return vec3(2.8 * (t - 0.5), 0.85,
                            0.35 - 1.2 * (t - 0.5));
  return vec3(0.70 + 1.2 * (t - 0.75),
              0.85 - 2.2 * (t - 0.75), 0.05);
}

void main() {
  if (uUnlit) {
    fragColor = vec4(vColor, 1.0);
    return;
  }
  if (uShellHeat) {
    float irradiance = texelFetch(uShellIrradiance, gl_PrimitiveID).r;
    if (irradiance >= 0.0) {
      fragColor = vec4(irradianceColor(irradiance), 1.0);
      return;
    }
  }
  vec3 normal = normalize(cross(dFdx(vWorld), dFdy(vWorld)));
  float incidence = abs(dot(normal, normalize(uSunDirection)));
  float lighting = uCellPass ? 0.78 + 0.22 * incidence
                             : 0.24 + 0.76 * incidence;
  fragColor = vec4(vColor * lighting, 1.0);
}
)glsl";

struct RenderVertex {
  Vec3 position;
  Vec3 color;
};

struct Camera {
  Vec3 target;
  float home_distance = 4.0f;
  float distance = 4.0f;
  float yaw = -1.15f;
  float pitch = 0.55f;
  bool dragging = false;
  double last_x = 0.0;
  double last_y = 0.0;

  void reset() {
    distance = home_distance;
    yaw = -1.15f;
    pitch = 0.55f;
  }
};

struct Breakdown {
  float direct = 0.0f;
  float sky = 0.0f;
  float reflected = 0.0f;
};

GLuint compile_shader(GLenum type, const char* source) {
  const GLuint shader = glCreateShader(type);
  glShaderSource(shader, 1, &source, nullptr);
  glCompileShader(shader);
  GLint okay = GL_FALSE;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &okay);
  if (!okay) {
    GLint length = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
    std::string log(static_cast<std::size_t>(length), '\0');
    glGetShaderInfoLog(shader, length, nullptr, log.data());
    glDeleteShader(shader);
    throw std::runtime_error("OpenGL shader compile failed: " + log);
  }
  return shader;
}

GLuint make_program() {
  const GLuint vertex = compile_shader(GL_VERTEX_SHADER, kVertexShader);
  const GLuint fragment = compile_shader(GL_FRAGMENT_SHADER, kFragmentShader);
  const GLuint program = glCreateProgram();
  glAttachShader(program, vertex);
  glAttachShader(program, fragment);
  glLinkProgram(program);
  glDeleteShader(vertex);
  glDeleteShader(fragment);
  GLint okay = GL_FALSE;
  glGetProgramiv(program, GL_LINK_STATUS, &okay);
  if (!okay) {
    GLint length = 0;
    glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
    std::string log(static_cast<std::size_t>(length), '\0');
    glGetProgramInfoLog(program, length, nullptr, log.data());
    glDeleteProgram(program);
    throw std::runtime_error("OpenGL shader link failed: " + log);
  }
  return program;
}

std::array<Vec3, 4> cell_corners(const Cell& cell) {
  const Vec3 u = cell.u_axis * (0.5f * cell.width_m);
  const Vec3 v = cell.v_axis * (0.5f * cell.height_m);
  return {cell.center - u - v, cell.center + u - v,
          cell.center + u + v, cell.center - u + v};
}

std::vector<RenderVertex> cell_vertices(const std::vector<Cell>& cells) {
  std::vector<RenderVertex> vertices;
  vertices.reserve(cells.size() * 6U);
  for (const Cell& cell : cells) {
    const auto corners = cell_corners(cell);
    const Vec3 color = irradiance_color(cell.total_w_m2());
    for (const int index : {0, 1, 2, 0, 2, 3})
      vertices.push_back({corners[static_cast<std::size_t>(index)], color});
  }
  return vertices;
}

std::vector<RenderVertex> cell_outlines(const std::vector<Cell>& cells) {
  std::vector<RenderVertex> vertices;
  vertices.reserve(cells.size() * 8U);
  const Vec3 color{0.015f, 0.025f, 0.035f};
  for (const Cell& cell : cells) {
    const auto corners = cell_corners(cell);
    for (const int index : {0, 1, 1, 2, 2, 3, 3, 0})
      vertices.push_back({corners[static_cast<std::size_t>(index)], color});
  }
  return vertices;
}

std::vector<RenderVertex> ground_grid(const Bounds& bounds) {
  std::vector<RenderVertex> vertices;
  const float half_size = std::max(
      5.0f, std::ceil(std::max(bounds.extent().x, bounds.extent().y) * 0.8f));
  const float spacing = 0.5f;
  const float x_center = bounds.center().x;
  const float y_center = bounds.center().y;
  const Vec3 minor{0.075f, 0.105f, 0.135f};
  for (float offset = -half_size; offset <= half_size + 0.01f;
       offset += spacing) {
    vertices.push_back({{x_center - half_size, y_center + offset, 0.0f}, minor});
    vertices.push_back({{x_center + half_size, y_center + offset, 0.0f}, minor});
    vertices.push_back({{x_center + offset, y_center - half_size, 0.0f}, minor});
    vertices.push_back({{x_center + offset, y_center + half_size, 0.0f}, minor});
  }
  vertices.push_back(
      {{x_center - half_size, y_center, 0.001f}, {0.60f, 0.18f, 0.18f}});
  vertices.push_back(
      {{x_center + half_size, y_center, 0.001f}, {0.60f, 0.18f, 0.18f}});
  vertices.push_back(
      {{x_center, y_center - half_size, 0.001f}, {0.18f, 0.52f, 0.32f}});
  vertices.push_back(
      {{x_center, y_center + half_size, 0.001f}, {0.18f, 0.52f, 0.32f}});
  return vertices;
}

std::array<RenderVertex, 2> sun_vertices(const Bounds& bounds,
                                         const Vec3& direction) {
  const float length =
      std::max({bounds.extent().x, bounds.extent().y, bounds.extent().z}) *
      0.8f;
  return {RenderVertex{bounds.center(), {1.0f, 0.56f, 0.08f}},
          RenderVertex{bounds.center() + direction * length,
                       {1.0f, 0.92f, 0.35f}}};
}

void configure_render_vao(GLuint vao, GLuint vbo) {
  glBindVertexArray(vao);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(RenderVertex),
                        reinterpret_cast<void*>(offsetof(RenderVertex, position)));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(RenderVertex),
                        reinterpret_cast<void*>(offsetof(RenderVertex, color)));
}

void upload_vertices(GLuint vbo, const std::vector<RenderVertex>& vertices,
                     GLenum usage = GL_DYNAMIC_DRAW) {
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER,
               static_cast<GLsizeiptr>(vertices.size() * sizeof(RenderVertex)),
               vertices.data(), usage);
}

glm::vec3 as_glm(const Vec3& value) { return {value.x, value.y, value.z}; }

bool imgui_captures_mouse() {
  return ImGui::GetCurrentContext() && ImGui::GetIO().WantCaptureMouse;
}

void cursor_callback(GLFWwindow* window, double x, double y) {
  auto* camera = static_cast<Camera*>(glfwGetWindowUserPointer(window));
  if (!camera || !camera->dragging || imgui_captures_mouse()) return;
  const double dx = x - camera->last_x;
  const double dy = y - camera->last_y;
  camera->last_x = x;
  camera->last_y = y;
  camera->yaw += static_cast<float>(dx) * 0.006f;
  camera->pitch = std::clamp(camera->pitch - static_cast<float>(dy) * 0.006f,
                             -1.45f, 1.45f);
}

void mouse_callback(GLFWwindow* window, int button, int action, int) {
  if (button != GLFW_MOUSE_BUTTON_LEFT) return;
  auto* camera = static_cast<Camera*>(glfwGetWindowUserPointer(window));
  if (!camera) return;
  if (action == GLFW_RELEASE) {
    camera->dragging = false;
    return;
  }
  if (imgui_captures_mouse()) return;
  camera->dragging = true;
  glfwGetCursorPos(window, &camera->last_x, &camera->last_y);
}

void scroll_callback(GLFWwindow* window, double, double y_offset) {
  auto* camera = static_cast<Camera*>(glfwGetWindowUserPointer(window));
  if (!camera || imgui_captures_mouse()) return;
  const float steps =
      std::clamp(static_cast<float>(y_offset), -2.0f, 2.0f);
  camera->distance *= std::exp(-0.075f * steps);
  camera->distance =
      std::clamp(camera->distance, camera->home_distance * 0.12f,
                 camera->home_distance * 6.0f);
}

void glfw_error(int, const char* description) {
  std::cerr << "GLFW: " << description << '\n';
}

void set_gui_theme() {
  ImGui::StyleColorsDark();
  ImGuiStyle& style = ImGui::GetStyle();
  style.WindowRounding = 0.0f;
  style.ChildRounding = 8.0f;
  style.FrameRounding = 6.0f;
  style.GrabRounding = 6.0f;
  style.ScrollbarRounding = 8.0f;
  style.WindowPadding = {18.0f, 16.0f};
  style.FramePadding = {10.0f, 7.0f};
  style.ItemSpacing = {9.0f, 9.0f};
  style.ItemInnerSpacing = {7.0f, 5.0f};
  style.WindowBorderSize = 0.0f;
  style.ChildBorderSize = 1.0f;

  auto& colors = style.Colors;
  colors[ImGuiCol_WindowBg] = {0.035f, 0.050f, 0.070f, 0.985f};
  colors[ImGuiCol_ChildBg] = {0.055f, 0.075f, 0.100f, 1.0f};
  colors[ImGuiCol_Border] = {0.12f, 0.17f, 0.22f, 1.0f};
  colors[ImGuiCol_FrameBg] = {0.075f, 0.100f, 0.130f, 1.0f};
  colors[ImGuiCol_FrameBgHovered] = {0.10f, 0.15f, 0.19f, 1.0f};
  colors[ImGuiCol_FrameBgActive] = {0.12f, 0.19f, 0.24f, 1.0f};
  colors[ImGuiCol_Button] = {0.04f, 0.52f, 0.70f, 1.0f};
  colors[ImGuiCol_ButtonHovered] = {0.05f, 0.64f, 0.82f, 1.0f};
  colors[ImGuiCol_ButtonActive] = {0.03f, 0.42f, 0.58f, 1.0f};
  colors[ImGuiCol_CheckMark] = {0.15f, 0.82f, 0.95f, 1.0f};
  colors[ImGuiCol_SliderGrab] = {0.10f, 0.67f, 0.84f, 1.0f};
  colors[ImGuiCol_SliderGrabActive] = {0.25f, 0.82f, 0.95f, 1.0f};
  colors[ImGuiCol_Header] = {0.06f, 0.39f, 0.52f, 1.0f};
  colors[ImGuiCol_Text] = {0.91f, 0.94f, 0.97f, 1.0f};
  colors[ImGuiCol_TextDisabled] = {0.49f, 0.57f, 0.64f, 1.0f};
}

Breakdown calculate_breakdown(const std::vector<Cell>& cells) {
  Breakdown result;
  float area = 0.0f;
  for (const Cell& cell : cells) {
    result.direct += cell.direct_w_m2 * cell.active_area_m2;
    result.sky += cell.diffuse_sky_w_m2 * cell.active_area_m2;
    result.reflected += cell.scene_reflected_w_m2 * cell.active_area_m2;
    area += cell.active_area_m2;
  }
  if (area > 0.0f) {
    result.direct /= area;
    result.sky /= area;
    result.reflected /= area;
  }
  return result;
}

void draw_label_value(const char* label, const std::string& value) {
  const float right_edge =
      ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x;
  ImGui::TextDisabled("%s", label);
  ImGui::SameLine();
  ImGui::SetCursorPosX(
      std::max(ImGui::GetCursorPosX(),
               right_edge - ImGui::CalcTextSize(value.c_str()).x));
  ImGui::TextUnformatted(value.c_str());
}

void draw_heat_legend() {
  ImGui::TextDisabled("IRRADIANCE HEAT MAP");
  const ImVec2 start = ImGui::GetCursorScreenPos();
  const float width = ImGui::GetContentRegionAvail().x;
  constexpr float height = 13.0f;
  ImDrawList* draw = ImGui::GetWindowDrawList();
  constexpr int segments = 80;
  for (int index = 0; index < segments; ++index) {
    const float t0 = static_cast<float>(index) / segments;
    const float t1 = static_cast<float>(index + 1) / segments;
    const Vec3 color = irradiance_color(t0 * 1000.0f);
    draw->AddRectFilled({start.x + t0 * width, start.y},
                        {start.x + t1 * width + 1.0f, start.y + height},
                        ImGui::ColorConvertFloat4ToU32(
                            {color.x, color.y, color.z, 1.0f}));
  }
  draw->AddRect(start, {start.x + width, start.y + height},
                IM_COL32(70, 85, 100, 255), 3.0f);
  ImGui::Dummy({width, height + 2.0f});
  ImGui::TextDisabled("0");
  ImGui::SameLine(width * 0.43f);
  ImGui::TextDisabled("500");
  ImGui::SameLine(width - 49.0f);
  ImGui::TextDisabled("1000 W/m2");
}

std::string fixed(float value, int precision = 1) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

}  // namespace

void show_viewer(const TriangleMesh& mesh, const TraceScene& scene,
                 std::vector<Cell>& cells, Sun& sun,
                 SimulationSettings& settings, SimulationSummary& summary,
                 std::vector<float>& shell_irradiance,
                 const std::filesystem::path& output_path) {
  glfwSetErrorCallback(glfw_error);
  if (!glfwInit()) throw std::runtime_error("GLFW initialization failed");
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_SAMPLES, 4);
#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif
  GLFWwindow* window = glfwCreateWindow(
      1600, 960, "Solar Array Irradiance Studio", nullptr, nullptr);
  if (!window) {
    glfwTerminate();
    throw std::runtime_error("Could not create an OpenGL window");
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  glewExperimental = GL_TRUE;
  const GLenum glew_status = glewInit();
  glGetError();
  if (glew_status != GLEW_OK) {
    glfwDestroyWindow(window);
    glfwTerminate();
    throw std::runtime_error("GLEW initialization failed");
  }

  GLuint shell_vao = 0, shell_vbo = 0;
  GLuint shell_heat_buffer = 0, shell_heat_texture = 0;
  GLuint cells_vao = 0, cells_vbo = 0;
  GLuint outlines_vao = 0, outlines_vbo = 0;
  GLuint grid_vao = 0, grid_vbo = 0;
  GLuint sun_vao = 0, sun_vbo = 0;
  GLuint program = 0;
  bool imgui_initialized = false;

  try {
    program = make_program();

    glGenVertexArrays(1, &shell_vao);
    glGenBuffers(1, &shell_vbo);
    glBindVertexArray(shell_vao);
    glBindBuffer(GL_ARRAY_BUFFER, shell_vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(mesh.vertices.size() * sizeof(Vec3)),
                 mesh.vertices.data(), GL_STATIC_DRAW);
    if (glGetError() == GL_OUT_OF_MEMORY)
      throw std::runtime_error(
          "GPU could not hold the complete STL. Run with --headless or export "
          "a topology-preserving display mesh while retaining the full trace mesh.");
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), nullptr);
    glDisableVertexAttribArray(1);
    glVertexAttrib3f(1, 0.30f, 0.36f, 0.42f);

    if (shell_irradiance.size() != mesh.triangle_count())
      throw std::runtime_error("Shell heat map size does not match STL triangles");
    GLint max_texture_buffer_texels = 0;
    glGetIntegerv(GL_MAX_TEXTURE_BUFFER_SIZE, &max_texture_buffer_texels);
    if (shell_irradiance.size() >
        static_cast<std::size_t>(max_texture_buffer_texels)) {
      throw std::runtime_error(
          "GPU texture-buffer limit is too small for the shell heat map");
    }
    glGenBuffers(1, &shell_heat_buffer);
    glBindBuffer(GL_TEXTURE_BUFFER, shell_heat_buffer);
    glBufferData(GL_TEXTURE_BUFFER,
                 static_cast<GLsizeiptr>(shell_irradiance.size() *
                                         sizeof(float)),
                 shell_irradiance.data(), GL_DYNAMIC_DRAW);
    if (glGetError() == GL_OUT_OF_MEMORY)
      throw std::runtime_error(
          "GPU could not hold the per-triangle shell heat map");
    glGenTextures(1, &shell_heat_texture);
    glBindTexture(GL_TEXTURE_BUFFER, shell_heat_texture);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, shell_heat_buffer);

    std::vector<RenderVertex> rendered_cells = cell_vertices(cells);
    std::vector<RenderVertex> rendered_outlines = cell_outlines(cells);
    const std::vector<RenderVertex> rendered_grid = ground_grid(mesh.bounds);

    glGenVertexArrays(1, &cells_vao);
    glGenBuffers(1, &cells_vbo);
    configure_render_vao(cells_vao, cells_vbo);
    upload_vertices(cells_vbo, rendered_cells);

    glGenVertexArrays(1, &outlines_vao);
    glGenBuffers(1, &outlines_vbo);
    configure_render_vao(outlines_vao, outlines_vbo);
    upload_vertices(outlines_vbo, rendered_outlines);

    glGenVertexArrays(1, &grid_vao);
    glGenBuffers(1, &grid_vbo);
    configure_render_vao(grid_vao, grid_vbo);
    upload_vertices(grid_vbo, rendered_grid, GL_STATIC_DRAW);

    glGenVertexArrays(1, &sun_vao);
    glGenBuffers(1, &sun_vbo);
    configure_render_vao(sun_vao, sun_vbo);
    auto rendered_sun = sun_vertices(mesh.bounds, sun.direction_to_sun);
    glBindBuffer(GL_ARRAY_BUFFER, sun_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(rendered_sun), rendered_sun.data(),
                 GL_DYNAMIC_DRAW);

    Camera camera;
    camera.target = mesh.bounds.center();
    camera.home_distance =
        1.65f * std::max({mesh.bounds.extent().x, mesh.bounds.extent().y,
                          mesh.bounds.extent().z});
    camera.reset();
    glfwSetWindowUserPointer(window, &camera);
    glfwSetCursorPosCallback(window, cursor_callback);
    glfwSetMouseButtonCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    set_gui_theme();
    if (!ImGui_ImplGlfw_InitForOpenGL(window, true) ||
        !ImGui_ImplOpenGL3_Init("#version 330"))
      throw std::runtime_error("Dear ImGui backend initialization failed");
    imgui_initialized = true;

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glDisable(GL_CULL_FACE);

    float azimuth =
        std::atan2(sun.direction_to_sun.x, sun.direction_to_sun.y) * 180.0f /
        kPi;
    if (azimuth < 0.0f) azimuth += 360.0f;
    float elevation = std::asin(std::clamp(sun.direction_to_sun.z, -1.0f, 1.0f)) *
                      180.0f / kPi;
    int ray_count = static_cast<int>(settings.hemisphere_samples);
    int reflection_depth = static_cast<int>(settings.max_reflection_depth);
    bool show_shell = true;
    bool show_shell_heat = true;
    bool show_cells = false;
    bool show_grid = true;
    bool show_sun = true;
    bool wireframe = false;
    bool inputs_dirty = false;
    std::string status = "Calculating shell heat map...";
    Breakdown breakdown = calculate_breakdown(cells);
    std::optional<float> hovered_shell_irradiance;
    auto shell_heat_started = std::chrono::steady_clock::now();
    std::future<std::vector<float>> shell_heat_future = std::async(
        std::launch::async, [&mesh, &scene, sun, settings] {
          return simulate_top_shell_irradiance(mesh, scene, sun, settings);
        });

    while (!glfwWindowShouldClose(window)) {
      glfwPollEvents();
      if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);

      if (shell_heat_future.valid() &&
          shell_heat_future.wait_for(std::chrono::milliseconds(0)) ==
              std::future_status::ready) {
        shell_irradiance = shell_heat_future.get();
        glBindBuffer(GL_TEXTURE_BUFFER, shell_heat_buffer);
        glBufferSubData(GL_TEXTURE_BUFFER, 0,
                        static_cast<GLsizeiptr>(shell_irradiance.size() *
                                                sizeof(float)),
                        shell_irradiance.data());
        const double seconds = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() -
                                   shell_heat_started)
                                   .count();
        status = "Shell heat map ready in " +
                 fixed(static_cast<float>(seconds), 2) + " s";
      }

      ImGui_ImplOpenGL3_NewFrame();
      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();

      ImGui::SetNextWindowPos({0.0f, 0.0f});
      ImGui::SetNextWindowSize({kPanelWidth, io.DisplaySize.y});
      const ImGuiWindowFlags panel_flags =
          ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove |
          ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse;
      ImGui::Begin("Simulation controls", nullptr, panel_flags);

      ImGui::TextColored({0.18f, 0.80f, 0.96f, 1.0f}, "SOLAR WHITTED");
      ImGui::SameLine();
      ImGui::TextDisabled("FULL-MESH STUDIO");
      ImGui::Text("Solar array irradiance");
      ImGui::TextDisabled("Embree physics + OpenGL visualization");
      ImGui::Spacing();

      ImGui::BeginChild("Summary", {0.0f, 134.0f}, true);
      ImGui::TextDisabled("CURRENT RESULT");
      ImGui::TextColored({0.98f, 0.78f, 0.24f, 1.0f}, "%s W/m2",
                         fixed(summary.area_weighted_irradiance_w_m2, 1).c_str());
      ImGui::SameLine();
      ImGui::TextDisabled("area weighted");
      draw_label_value("Incident optical power",
                       fixed(summary.incident_solar_power_w, 1) + " W");
      draw_label_value("Accepted cells", std::to_string(summary.cell_count));
      draw_label_value("Active area", fixed(summary.active_area_m2, 3) + " m2");
      ImGui::EndChild();

      ImGui::Spacing();
      ImGui::TextDisabled("SUN & SKY");
      inputs_dirty |= ImGui::SliderFloat("Azimuth", &azimuth, 0.0f, 360.0f,
                                         "%.1f deg");
      inputs_dirty |= ImGui::SliderFloat("Elevation", &elevation, -5.0f, 90.0f,
                                         "%.1f deg");
      inputs_dirty |=
          ImGui::SliderFloat("DNI", &sun.dni_w_m2, 0.0f, 1200.0f, "%.0f W/m2");
      inputs_dirty |=
          ImGui::SliderFloat("DHI", &sun.dhi_w_m2, 0.0f, 600.0f, "%.0f W/m2");
      sun.direction_to_sun =
          sun_direction_from_azimuth_elevation(azimuth, elevation);

      ImGui::Spacing();
      ImGui::TextDisabled("RAY TRACING");
      if (ImGui::SliderInt("Sky rays / cell", &ray_count, 8, 512)) {
        ray_count = std::max(8, ray_count);
        inputs_dirty = true;
      }
      if (ImGui::SliderInt("Reflection depth", &reflection_depth, 0, 4))
        inputs_dirty = true;
      settings.hemisphere_samples = static_cast<std::uint32_t>(ray_count);
      settings.max_reflection_depth =
          static_cast<std::uint32_t>(reflection_depth);

      if (inputs_dirty)
        ImGui::TextColored({0.98f, 0.67f, 0.20f, 1.0f},
                           "Inputs changed - run to refresh heat map");
      else
        ImGui::TextColored({0.35f, 0.80f, 0.55f, 1.0f}, "%s", status.c_str());

      const bool shell_heat_busy = shell_heat_future.valid();
      if (shell_heat_busy) ImGui::BeginDisabled();
      if (ImGui::Button("RUN SIMULATION", {-1.0f, 42.0f}) &&
          !shell_heat_busy) {
        const auto start = std::chrono::steady_clock::now();
        summary = simulate_cells(cells, scene, sun, settings);
        breakdown = calculate_breakdown(cells);
        rendered_cells = cell_vertices(cells);
        rendered_outlines = cell_outlines(cells);
        upload_vertices(cells_vbo, rendered_cells);
        upload_vertices(outlines_vbo, rendered_outlines);
        write_cell_csv(output_path, cells);
        const double seconds = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() - start)
                                   .count();
        const Sun shell_sun = sun;
        const SimulationSettings shell_settings = settings;
        shell_heat_started = std::chrono::steady_clock::now();
        shell_heat_future = std::async(
            std::launch::async,
            [&mesh, &scene, shell_sun, shell_settings] {
              return simulate_top_shell_irradiance(
                  mesh, scene, shell_sun, shell_settings);
            });
        status = "Cells completed in " +
                 fixed(static_cast<float>(seconds), 2) +
                 " s; calculating shell...";
        inputs_dirty = false;
      }
      if (shell_heat_busy) ImGui::EndDisabled();
      if (ImGui::Button("Export current CSV", {-1.0f, 0.0f})) {
        write_cell_csv(output_path, cells);
        status = "CSV exported";
      }
      if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", output_path.string().c_str());

      ImGui::Spacing();
      ImGui::TextDisabled("IRRADIANCE BREAKDOWN");
      draw_label_value("Direct sun", fixed(breakdown.direct, 1) + " W/m2");
      draw_label_value("Diffuse sky", fixed(breakdown.sky, 1) + " W/m2");
      draw_label_value("Scene reflection",
                       fixed(breakdown.reflected, 1) + " W/m2");
      ImGui::Spacing();
      draw_heat_legend();
      if (hovered_shell_irradiance) {
        draw_label_value("Shell under cursor",
                         fixed(*hovered_shell_irradiance, 1) + " W/m2");
      } else if (shell_heat_busy) {
        ImGui::TextDisabled("Shell values are calculating...");
      } else {
        ImGui::TextDisabled("Hover over the top shell for exact W/m2");
      }

      ImGui::Spacing();
      ImGui::TextDisabled("VIEW");
      ImGui::Checkbox("Shell", &show_shell);
      ImGui::SameLine(112.0f);
      ImGui::Checkbox("Grid", &show_grid);
      ImGui::SameLine(220.0f);
      ImGui::Checkbox("Wireframe", &wireframe);
      ImGui::Checkbox("Shell heat map", &show_shell_heat);
      ImGui::Checkbox("Candidate cells", &show_cells);
      ImGui::SameLine(180.0f);
      ImGui::Checkbox("Sun direction", &show_sun);
      if (ImGui::Button("Reset camera", {-1.0f, 0.0f})) camera.reset();

      ImGui::Spacing();
      ImGui::Separator();
      ImGui::TextDisabled("%zu triangles  |  drag to orbit  |  wheel to zoom",
                          mesh.triangle_count());
      ImGui::TextDisabled("Optical irradiance only - no electrical model");
      ImGui::End();

      int framebuffer_width = 1, framebuffer_height = 1;
      int window_width = 1, window_height = 1;
      glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
      glfwGetWindowSize(window, &window_width, &window_height);
      const float pixel_scale = static_cast<float>(framebuffer_width) /
                                static_cast<float>(std::max(1, window_width));
      const int viewport_x =
          std::min(framebuffer_width - 1, static_cast<int>(kPanelWidth * pixel_scale));
      const int viewport_width = std::max(1, framebuffer_width - viewport_x);

      glViewport(0, 0, framebuffer_width, framebuffer_height);
      glClearColor(0.012f, 0.023f, 0.036f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      glViewport(viewport_x, 0, viewport_width, framebuffer_height);

      const float cp = std::cos(camera.pitch);
      const glm::vec3 eye = as_glm(camera.target) +
                            camera.distance * glm::vec3(
                                cp * std::cos(camera.yaw),
                                cp * std::sin(camera.yaw),
                                std::sin(camera.pitch));
      const glm::mat4 view = glm::lookAt(
          eye, as_glm(camera.target), glm::vec3(0.0f, 0.0f, 1.0f));
      const glm::mat4 projection = glm::perspective(
          glm::radians(43.0f), static_cast<float>(viewport_width) /
                                  static_cast<float>(std::max(1, framebuffer_height)),
          0.01f, 250.0f);
      const glm::mat4 vp = projection * view;

      hovered_shell_irradiance.reset();
      double cursor_x = 0.0, cursor_y = 0.0;
      glfwGetCursorPos(window, &cursor_x, &cursor_y);
      const double viewport_window_width =
          static_cast<double>(window_width) - kPanelWidth;
      if (!io.WantCaptureMouse && cursor_x >= kPanelWidth &&
          cursor_x < window_width && cursor_y >= 0.0 &&
          cursor_y < window_height && viewport_window_width > 1.0) {
        const float ndc_x = static_cast<float>(
            2.0 * (cursor_x - kPanelWidth) / viewport_window_width - 1.0);
        const float ndc_y =
            static_cast<float>(1.0 - 2.0 * cursor_y / window_height);
        const glm::mat4 inverse_vp = glm::inverse(vp);
        glm::vec4 near_point = inverse_vp * glm::vec4(ndc_x, ndc_y, -1.0f, 1.0f);
        glm::vec4 far_point = inverse_vp * glm::vec4(ndc_x, ndc_y, 1.0f, 1.0f);
        near_point /= near_point.w;
        far_point /= far_point.w;
        const glm::vec3 ray = glm::normalize(
            glm::vec3(far_point) - glm::vec3(near_point));
        const auto hit = scene.intersect(
            {eye.x, eye.y, eye.z}, {ray.x, ray.y, ray.z},
            settings.ray_epsilon_m, 1.0e30f, kCarMask);
        if (hit && hit->primitive_id < shell_irradiance.size()) {
          const float value = shell_irradiance[hit->primitive_id];
          if (value >= 0.0f) hovered_shell_irradiance = value;
        }
      }

      glUseProgram(program);
      glUniformMatrix4fv(glGetUniformLocation(program, "uViewProjection"), 1,
                         GL_FALSE, glm::value_ptr(vp));
      glUniform3f(glGetUniformLocation(program, "uSunDirection"),
                  sun.direction_to_sun.x, sun.direction_to_sun.y,
                  sun.direction_to_sun.z);
      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_BUFFER, shell_heat_texture);
      glUniform1i(glGetUniformLocation(program, "uShellIrradiance"), 0);

      glUniform1i(glGetUniformLocation(program, "uUnlit"), GL_TRUE);
      glUniform1i(glGetUniformLocation(program, "uCellPass"), GL_FALSE);
      glUniform1i(glGetUniformLocation(program, "uShellHeat"), GL_FALSE);
      if (show_grid) {
        glBindVertexArray(grid_vao);
        glLineWidth(1.0f);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(rendered_grid.size()));
      }

      if (show_shell) {
        glUniform1i(glGetUniformLocation(program, "uUnlit"), GL_FALSE);
        glUniform1i(glGetUniformLocation(program, "uCellPass"), GL_FALSE);
        glUniform1i(glGetUniformLocation(program, "uShellHeat"),
                    show_shell_heat ? GL_TRUE : GL_FALSE);
        glBindVertexArray(shell_vao);
        if (wireframe) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDrawArrays(GL_TRIANGLES, 0,
                     static_cast<GLsizei>(mesh.vertices.size()));
        if (wireframe) glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
      }

      if (show_cells) {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(-1.0f, -1.0f);
        glUniform1i(glGetUniformLocation(program, "uUnlit"), GL_FALSE);
        glUniform1i(glGetUniformLocation(program, "uCellPass"), GL_TRUE);
        glUniform1i(glGetUniformLocation(program, "uShellHeat"), GL_FALSE);
        glBindVertexArray(cells_vao);
        glDrawArrays(GL_TRIANGLES, 0,
                     static_cast<GLsizei>(rendered_cells.size()));
        glDisable(GL_POLYGON_OFFSET_FILL);
        glUniform1i(glGetUniformLocation(program, "uUnlit"), GL_TRUE);
        glBindVertexArray(outlines_vao);
        glLineWidth(1.0f);
        glDrawArrays(GL_LINES, 0,
                     static_cast<GLsizei>(rendered_outlines.size()));
      }

      if (show_sun) {
        rendered_sun = sun_vertices(mesh.bounds, sun.direction_to_sun);
        glBindBuffer(GL_ARRAY_BUFFER, sun_vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(rendered_sun),
                        rendered_sun.data());
        glUniform1i(glGetUniformLocation(program, "uUnlit"), GL_TRUE);
        glBindVertexArray(sun_vao);
        glLineWidth(4.0f);
        glDrawArrays(GL_LINES, 0, 2);
      }

      ImGui::Render();
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
      glfwSwapBuffers(window);
    }
  } catch (...) {
    if (imgui_initialized) {
      ImGui_ImplOpenGL3_Shutdown();
      ImGui_ImplGlfw_Shutdown();
      ImGui::DestroyContext();
    }
    if (program) glDeleteProgram(program);
    if (shell_heat_texture) glDeleteTextures(1, &shell_heat_texture);
    if (shell_heat_buffer) glDeleteBuffers(1, &shell_heat_buffer);
    for (GLuint buffer : {shell_vbo, cells_vbo, outlines_vbo, grid_vbo, sun_vbo})
      if (buffer) glDeleteBuffers(1, &buffer);
    for (GLuint array : {shell_vao, cells_vao, outlines_vao, grid_vao, sun_vao})
      if (array) glDeleteVertexArrays(1, &array);
    glfwDestroyWindow(window);
    glfwTerminate();
    throw;
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glDeleteProgram(program);
  glDeleteTextures(1, &shell_heat_texture);
  glDeleteBuffers(1, &shell_heat_buffer);
  for (GLuint buffer : {shell_vbo, cells_vbo, outlines_vbo, grid_vbo, sun_vbo})
    glDeleteBuffers(1, &buffer);
  for (GLuint array : {shell_vao, cells_vao, outlines_vao, grid_vao, sun_vao})
    glDeleteVertexArrays(1, &array);
  glfwDestroyWindow(window);
  glfwTerminate();
}

}  // namespace solar
