#include "solar/viewer.hpp"
#include "solar/weather.hpp"
#include "solar/optimizer.hpp"
#include "solar/race.hpp"
#include "solar/placement_surface.hpp"

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
#include <cstring>
#include <future>
#include <memory>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#ifdef _WIN32
#include <windows.h>
#include <commdlg.h>
#endif

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
uniform float uDni;
uniform float uDhi;
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
  vec3 normal = normalize(cross(dFdx(vWorld), dFdy(vWorld)));
  if (normal.z < 0.0) normal = -normal;
  if (uShellHeat) {
    float direct = uDni * max(0.0, dot(normal, normalize(uSunDirection)));
    float sky = uDhi * clamp((1.0 + normal.z) * 0.5, 0.0, 1.0);
    // Preserve enough form shading to make the curved STL readable beneath
    // the heat map. Irradiance still selects the hue; this factor only affects
    // the OpenGL preview and never feeds the Embree calculation.
    vec3 key = normalize(vec3(-0.35, -0.45, 0.82));
    float form = 0.74 + 0.26 * max(0.0, dot(normal, key));
    fragColor = vec4(irradianceColor(direct + sky) * form, 1.0);
    return;
  }
  float incidence = max(0.0, dot(normal, normalize(uSunDirection)));
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

struct CameraRouting {
  Camera* car = nullptr;
  Camera* track = nullptr;
  Camera* dragging = nullptr;
  bool split = false;
  double split_x = 0;
};

struct LoadedTopShell {
  PlacementSurface surface;
  TraceScene scene;
  std::filesystem::path path;
  LoadedTopShell(PlacementSurface prepared, std::filesystem::path file)
      : surface(std::move(prepared)), scene(surface.mesh), path(std::move(file)) {}
};

#ifdef _WIN32
std::string browse_stl() {
  wchar_t file[32768]{};
  OPENFILENAMEW dialog{};
  dialog.lStructSize = sizeof(dialog);
  dialog.hwndOwner = GetActiveWindow();
  dialog.lpstrFilter = L"STL files\0*.stl\0All files\0*.*\0";
  dialog.lpstrFile = file;
  dialog.nMaxFile = static_cast<DWORD>(std::size(file));
  dialog.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
  dialog.lpstrTitle = L"Select the assembly-aligned top-shell STL";
  if (!GetOpenFileNameW(&dialog)) return {};
  const int length = WideCharToMultiByte(CP_UTF8, 0, file, -1, nullptr, 0, nullptr, nullptr);
  if (length <= 1) return {};
  std::string result(static_cast<std::size_t>(length), '\0');
  WideCharToMultiByte(CP_UTF8, 0, file, -1, result.data(), length, nullptr, nullptr);
  result.pop_back();
  return result;
}
#endif

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
  for (const Cell& cell : cells) {
    const Vec3 color = cell.module_columns == 2U
                           ? Vec3{0.10f, 0.78f, 0.96f}
                           : Vec3{1.00f, 0.55f, 0.12f};
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
  auto* routing = static_cast<CameraRouting*>(glfwGetWindowUserPointer(window));
  auto* camera = routing ? routing->dragging : nullptr;
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
  auto* routing = static_cast<CameraRouting*>(glfwGetWindowUserPointer(window));
  if (!routing) return;
  if (action == GLFW_RELEASE) {
    if (routing->dragging) routing->dragging->dragging = false;
    routing->dragging = nullptr;
    return;
  }
  if (imgui_captures_mouse()) return;
  double x = 0, y = 0;
  glfwGetCursorPos(window, &x, &y);
  auto* camera = routing->split && x < routing->split_x ? routing->track : routing->car;
  routing->dragging = camera;
  camera->dragging = true;
  glfwGetCursorPos(window, &camera->last_x, &camera->last_y);
}

void scroll_callback(GLFWwindow* window, double, double y_offset) {
  auto* routing = static_cast<CameraRouting*>(glfwGetWindowUserPointer(window));
  if (!routing) return;
  double x = 0, y = 0; glfwGetCursorPos(window, &x, &y);
  auto* camera = routing->split && x < routing->split_x ? routing->track : routing->car;
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

float wrapped_degrees(float value) {
  value = std::fmod(value, 360.0f);
  return value < 0.0f ? value + 360.0f : value;
}

}  // namespace

void show_viewer(const TriangleMesh& mesh, const TraceScene& scene,
                 std::vector<Cell>& cells, Sun& sun,
                 SimulationSettings& settings, SimulationSummary& summary,
                 LayoutSummary& layout_summary,
                 const CellGrid& grid, const ShellRegion& region,
                 const ViewerGeometryInputs& geometry_inputs,
                 const std::filesystem::path& race_config_path,
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
  GLuint cells_vao = 0, cells_vbo = 0;
  GLuint outlines_vao = 0, outlines_vbo = 0;
  GLuint grid_vao = 0, grid_vbo = 0;
  GLuint sun_vao = 0, sun_vbo = 0;
  GLuint track_vao = 0, track_vbo = 0;
  GLuint top_vao = 0, top_vbo = 0;
  GLuint program = 0;
  bool imgui_initialized = false;
  bool imgui_context_created = false;
  bool imgui_glfw_initialized = false;

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

    glGenVertexArrays(1, &track_vao);
    glGenBuffers(1, &track_vbo);
    configure_render_vao(track_vao, track_vbo);
    glGenVertexArrays(1, &top_vao);
    glGenBuffers(1, &top_vbo);
    configure_render_vao(top_vao, top_vbo);

    Camera camera;
    camera.target = mesh.bounds.center();
    camera.home_distance =
        1.65f * std::max({mesh.bounds.extent().x, mesh.bounds.extent().y,
                          mesh.bounds.extent().z});
    camera.reset();
    Camera track_camera = camera;
    bool track_view = false;
    bool select_layout_tab = false;
    CameraRouting camera_routing{&camera, &track_camera};
    glfwSetWindowUserPointer(window, &camera_routing);
    glfwSetCursorPosCallback(window, cursor_callback);
    glfwSetMouseButtonCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    imgui_context_created = true;
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    set_gui_theme();
    if (!ImGui_ImplGlfw_InitForOpenGL(window, true))
      throw std::runtime_error("Dear ImGui GLFW backend initialization failed");
    imgui_glfw_initialized = true;
    if (!ImGui_ImplOpenGL3_Init("#version 330"))
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
    int direct_samples =
        static_cast<int>(settings.direct_samples_per_axis);
    int reflection_depth = static_cast<int>(settings.max_reflection_depth);
    bool show_shell = true;
    bool show_shell_heat = true;
    bool show_cells = true;
    bool show_cell_heat = false;
    bool show_grid = true;
    bool show_sun = true;
    bool wireframe = false;
    bool inputs_dirty = false;
    std::string status = "Simulation ready";
    Breakdown breakdown = calculate_breakdown(cells);
    std::optional<float> hovered_shell_irradiance;
    double weather_latitude = 0.0;
    double weather_longitude = 0.0;
    float vehicle_heading = 0.0f;
    std::optional<WeatherConditions> weather_conditions;
    std::future<WeatherConditions> weather_future;
    bool weather_request_running = false;
    std::string weather_status = "Enter coordinates, then fetch current conditions";
    struct RaceJob { OptimizedLayout optimized; RaceRun race; RaceConfig config; };
    std::future<RaceJob> race_future;
    bool race_running = false;
    bool race_average = false;
    std::optional<RaceJob> race_result;
    std::optional<RaceConfig> race_config;
    std::string race_status;
    char top_shell_path[4096]{};
    std::shared_ptr<LoadedTopShell> loaded_top;
    std::shared_ptr<LoadedTopShell> pending_top;
    Transform top_correction = geometry_inputs.top_correction;
    std::vector<RenderVertex> rendered_top;
    std::string top_status = "Load the separate top-shell STL before optimizing";
    const auto load_top_shell = [&] {
      loaded_top.reset();
      pending_top.reset();
      rendered_top.clear();
      race_result.reset();
      cells.clear(); summary = {}; layout_summary = {};
      rendered_cells.clear(); rendered_outlines.clear();
      upload_vertices(cells_vbo, rendered_cells);
      upload_vertices(outlines_vbo, rendered_outlines);
      breakdown = {};
      race_average = false;
      try {
        const auto path = std::filesystem::u8path(top_shell_path);
        auto source = load_binary_stl(path, geometry_inputs.transform);
        transform_mesh(source, top_correction);
        for (const auto& point : source.vertices)
          rendered_top.push_back({point, {0.85f, 0.42f, 0.10f}});
        upload_vertices(top_vbo, rendered_top, GL_STATIC_DRAW);
        camera.target = source.bounds.center();
        camera.home_distance = 1.65f * std::max({source.bounds.extent().x, source.bounds.extent().y, source.bounds.extent().z});
        camera.reset();
        auto prepared = prepare_placement_surface(source, scene,
            {.alignment_tolerance_m = 0.005f,
             .minimum_upward_normal_z = region.minimum_upward_normal_z});
        pending_top = std::make_shared<LoadedTopShell>(std::move(prepared), path);
        top_status = "Alignment checks passed. Inspect the orange overlay, then approve to enable optimization.";
      } catch (const std::exception& error) { top_status = error.what(); }
    };
    if (!geometry_inputs.top_shell_path.empty()) {
      const auto text = geometry_inputs.top_shell_path.u8string();
      if (text.size() < sizeof(top_shell_path)) {
        std::memcpy(top_shell_path, text.c_str(), text.size() + 1);
        load_top_shell();
      } else top_status = "Top-shell path is too long";
    }
    std::vector<RenderVertex> rendered_track;
    double track_length_m = 0;
    float track_relief_m = 0;
    std::string track_status;
    const auto load_track_view = [&] {
      rendered_track.clear();
      track_length_m = 0;
      track_relief_m = 0;
      track_status.clear();
      try {
        const auto track = load_track(race_config->track_path);
        const auto points = track_centerline_vertices(track);
        Bounds bounds;
        for (const auto& point : points) {
          bounds.include(point);
          rendered_track.push_back({point, {0.10f, 0.78f, 0.96f}});
        }
        track_length_m = track.surface_length_m;
        track_relief_m = bounds.extent().z;
        track_camera.target = bounds.center();
        track_camera.home_distance = 1.65f * std::max({bounds.extent().x, bounds.extent().y, bounds.extent().z, 1.0f});
        track_camera.reset();
        upload_vertices(track_vbo, rendered_track, GL_STATIC_DRAW);
      } catch (const std::exception& error) { track_status = error.what(); }
    };
    try { race_config = load_race_config(race_config_path); load_track_view(); }
    catch (const std::exception& error) { race_status = error.what(); }

    while (!glfwWindowShouldClose(window)) {
      glfwPollEvents();
      if (race_running && race_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        try {
          race_result = race_future.get();
          cells = race_result->optimized.layout.cells;
          layout_summary = race_result->optimized.layout.summary;
          summary = {};
          summary.cell_count = cells.size();
          for (const auto& cell : cells) summary.active_area_m2 += cell.active_area_m2;
          summary.incident_solar_power_w = static_cast<float>(race_result->optimized.optical_energy_wh / (race_result->race.duration_seconds / 3600.0));
          if (summary.active_area_m2 > 0) summary.area_weighted_irradiance_w_m2 = summary.incident_solar_power_w / summary.active_area_m2;
          breakdown = calculate_breakdown(cells);
          rendered_cells = cell_vertices(cells);
          rendered_outlines = cell_outlines(cells);
          upload_vertices(cells_vbo, rendered_cells);
          upload_vertices(outlines_vbo, rendered_outlines);
          race_average = true;
          inputs_dirty = false;
          status = "Race-average cell irradiance";
          write_cell_csv(output_path, cells);
          auto report = output_path; report.replace_extension("race.json");
          write_optimized_race(report, race_result->config, race_result->race, race_result->optimized);
          race_status = race_result->optimized.status;
        } catch (const std::exception& error) { race_status = error.what(); }
        race_running = false;
      }
      if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);

      if (weather_request_running &&
          weather_future.wait_for(std::chrono::seconds(0)) ==
              std::future_status::ready) {
        try {
          weather_conditions = weather_future.get();
          const WeatherConditions& conditions = *weather_conditions;
          sun.dni_w_m2 = conditions.dni_w_m2;
          sun.dhi_w_m2 = conditions.dhi_w_m2;
          azimuth = wrapped_degrees(conditions.sun.azimuth_degrees -
                                    vehicle_heading);
          elevation = conditions.sun.elevation_degrees;
          weather_status = "Applied " + conditions.observation_time_utc +
                           " UTC; run simulation to refresh cells";
          inputs_dirty = true;
        } catch (const std::exception& error) {
          weather_status = error.what();
        }
        weather_request_running = false;
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
      if (ImGui::BeginTabBar("Simulator views")) {
        if (ImGui::BeginTabItem("Car / array")) { track_view = false; ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("Track + layout", nullptr,
            select_layout_tab ? ImGuiTabItemFlags_SetSelected : 0)) {
          track_view = true; ImGui::EndTabItem();
        }
        select_layout_tab = false;
        ImGui::EndTabBar();
      }
      camera_routing.split = track_view;
      camera_routing.split_x = kPanelWidth + (io.DisplaySize.x - kPanelWidth) * 0.5;
      if (track_view) {
        ImGui::Text("Track: %.2f km | %zu vertices", track_length_m / 1000.0, rendered_track.size());
        ImGui::Text("Elevation range: %.1f m (true scale)", track_relief_m);
        ImGui::TextWrapped("Left: track centerline. Right: actual top shell, full-car overlay and selected cells. Orbit/zoom each pane independently.");
        if (!track_status.empty()) ImGui::TextWrapped("Track error: %s", track_status.c_str());
      }
      ImGui::Spacing();

      ImGui::TextDisabled("LOCAL GEOMETRY FILES");
      ImGui::TextWrapped("Full car: %s", geometry_inputs.car_path.string().c_str());
      ImGui::BeginDisabled(race_running);
      ImGui::SetNextItemWidth(-1.0f);
      bool correction_changed = ImGui::InputText("##top-shell-path", top_shell_path, sizeof(top_shell_path));
      ImGui::TextDisabled("Optional VERIFIED correction, after the common STL transform");
      correction_changed |= ImGui::InputFloat3("Shell offset (m)", &top_correction.translation.x, "%.4f");
      correction_changed |= ImGui::InputFloat3("Shell XYZ rotation (deg)", &top_correction.rotation_degrees.x, "%.3f");
      if (correction_changed) {
        loaded_top.reset(); pending_top.reset();
        cells.clear(); summary = {}; layout_summary = {}; race_result.reset(); breakdown = {};
        rendered_cells.clear(); rendered_outlines.clear(); rendered_top.clear();
        upload_vertices(cells_vbo, rendered_cells); upload_vertices(outlines_vbo, rendered_outlines);
        top_status = "File or correction changed; reload to validate. No automatic registration is applied.";
      }
#ifdef _WIN32
      if (ImGui::Button("Browse top-shell STL")) {
        const auto selected = browse_stl();
        if (!selected.empty() && selected.size() < sizeof(top_shell_path)) {
          std::memcpy(top_shell_path, selected.c_str(), selected.size() + 1);
          load_top_shell();
        }
      }
      ImGui::SameLine();
#endif
      if (ImGui::Button("Load top shell")) load_top_shell();
      ImGui::EndDisabled();
      ImGui::TextWrapped("%s", top_status.c_str());
      const auto checked_top = loaded_top ? loaded_top : pending_top;
      if (checked_top) {
        ImGui::TextWrapped("Validated file: %s", checked_top->path.string().c_str());
        ImGui::Text("Matched probes: %zu/%zu, max matched error %.2f mm",
            checked_top->surface.matched_samples, checked_top->surface.checked_samples,
            checked_top->surface.max_alignment_error_m * 1000.0);
        ImGui::TextWrapped("Unmatched shell regions are excluded from placement.");
      }
      if (pending_top && !loaded_top && !race_running && ImGui::Button("Approve validated shell alignment")) {
        loaded_top = pending_top;
        rendered_top.clear();
        for (const auto& point : loaded_top->surface.mesh.vertices)
          rendered_top.push_back({point, {0.24f, 0.46f, 0.54f}});
        upload_vertices(top_vbo, rendered_top, GL_STATIC_DRAW);
        top_status = "Approved shell loaded. Cells will be generated only on this matched surface.";
      }

      ImGui::BeginChild("Summary", {0.0f, 188.0f}, true);
      ImGui::TextDisabled("CURRENT RESULT");
      ImGui::TextColored({0.98f, 0.78f, 0.24f, 1.0f}, "%s W/m2",
                         fixed(summary.area_weighted_irradiance_w_m2, 1).c_str());
      ImGui::SameLine();
      ImGui::TextDisabled("area weighted");
      draw_label_value("Incident optical power",
                       fixed(summary.incident_solar_power_w, 1) + " W");
      draw_label_value("Selected cells", std::to_string(summary.cell_count));
      draw_label_value("2x3 modules",
                       std::to_string(layout_summary.two_by_three_modules));
      draw_label_value("1x3 modules",
                       std::to_string(layout_summary.one_by_three_modules));
      draw_label_value("Active area", fixed(summary.active_area_m2, 3) + " m2");
      ImGui::EndChild();

      ImGui::TextDisabled("TIMED TRACK LAYOUT OPTIMIZER");
      ImGui::TextWrapped("Config: %s", race_config_path.string().c_str());
      ImGui::BeginDisabled(race_running);
      if (ImGui::Button("Reload race config")) {
        try { race_config = load_race_config(race_config_path); race_status.clear(); load_track_view(); }
        catch (const std::exception& error) { race_config.reset(); race_status = error.what(); }
      }
      if (race_config) {
        ImGui::Text("%s  %s-%s %s", race_config->date.c_str(), race_config->start_local.c_str(), race_config->end_local.c_str(), race_config->timezone.c_str());
        ImGui::Text("Speed %.1f mph | efficiency %.1f%%", race_config->speed_mph, race_config->cell_efficiency * 100.0);
        ImGui::InputDouble("Max active area (m2)", &race_config->max_active_area_m2, 0.1, 1.0, "%.3f");
        ImGui::InputDouble("Position step (m)", &race_config->position_step_m, 0, 0, "%.3f");
        ImGui::InputDouble("Angle step (deg)", &race_config->angle_step_degrees, 0, 0, "%.1f");
        ImGui::InputDouble("Track step (m)", &race_config->track_step_m, 0, 0, "%.1f");
        ImGui::InputDouble("Packing solver limit (s)", &race_config->solver_time_limit_seconds, 0, 0, "%.0f");
        ImGui::InputDouble("Start offset on track (m)", &race_config->start_distance_m, 0, 0, "%.1f");
        if (ImGui::Button("Save race config")) {
          try { save_race_config(race_config_path, *race_config); race_status = "Race config saved"; }
          catch (const std::exception& error) { race_status = error.what(); }
        }
        ImGui::TextWrapped("Start offset 0 uses the supplied track origin; physical start/finish is not verified in the archive.");
        ImGui::BeginDisabled(!loaded_top);
        if (ImGui::Button("OPTIMIZE RACE LAYOUT", {-1.0f, 36.0f})) {
          const auto config = *race_config;
          const auto trace_settings = settings;
          const auto top_snapshot = loaded_top;
          race_future = std::async(std::launch::async, [&, config, trace_settings, top_snapshot] {
            RaceJob job;
            job.config = config;
            const auto track = load_track(config.track_path);
            job.race = build_race_run(config, track);
            job.optimized = optimize_race_layout(top_snapshot->scene,
                top_snapshot->surface.mesh.bounds, scene, grid,
                top_snapshot->surface.region, job.race, trace_settings, optimization_settings(config));
            return job;
          });
          race_running = true;
          track_view = true;
          select_layout_tab = true;
          race_status = "Fetching dated weather, scoring candidates, then packing. Scoring can be lengthy; the solver time limit applies only to packing.";
        }
        ImGui::EndDisabled();
      }
      ImGui::EndDisabled();
      ImGui::TextWrapped("%s", race_status.c_str());
      if (race_result) {
        const auto& result = race_result->optimized;
        ImGui::Text("Electrical energy: %.1f Wh", result.electrical_energy_wh);
        ImGui::Text("Optical energy: %.1f Wh", result.optical_energy_wh);
        ImGui::Text("Average electrical power: %.1f W", result.mean_electrical_power_w);
        ImGui::Text("%.2f laps | %zu time/track samples", race_result->race.laps, race_result->race.samples.size());
        ImGui::Text("Candidate modules: %zu", result.candidate_modules);
        if (result.relative_gap >= 0) ImGui::Text("Energy bound gap: %.2f%%", result.relative_gap * 100.0);
        else ImGui::TextUnformatted("Energy bound gap: unavailable");
        ImGui::TextWrapped("Finite-candidate solution only; not a continuous global optimum. Cell colors: %s.", race_average ? "race average" : "instantaneous rerun");
      }

      ImGui::Spacing();
      ImGui::TextDisabled("OPEN-METEO LOCATION");
      ImGui::SetNextItemWidth(-1.0f);
      ImGui::InputDouble("Latitude", &weather_latitude, 0.0, 0.0, "%.5f");
      ImGui::SetNextItemWidth(-1.0f);
      ImGui::InputDouble("Longitude", &weather_longitude, 0.0, 0.0, "%.5f");
      const bool heading_changed = ImGui::SliderFloat(
          "Vehicle heading", &vehicle_heading, 0.0f, 360.0f, "%.1f deg");
      if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Clockwise from true north; 0 is north, 90 is east");
      if (heading_changed && weather_conditions) {
        azimuth = wrapped_degrees(
            weather_conditions->sun.azimuth_degrees - vehicle_heading);
        inputs_dirty = true;
      }
      if (weather_request_running) ImGui::BeginDisabled();
      if (ImGui::Button(weather_request_running ? "FETCHING..."
                                                : "FETCH CURRENT WEATHER",
                        {-1.0f, 0.0f}) &&
          !weather_request_running) {
        try {
          const double latitude = weather_latitude;
          const double longitude = weather_longitude;
          weather_future = std::async(std::launch::async, [latitude, longitude] {
            return fetch_open_meteo_current(latitude, longitude);
          });
          weather_request_running = true;
          weather_status = "Contacting Open-Meteo...";
        } catch (const std::exception& error) {
          weather_status = error.what();
        }
      }
      if (weather_request_running) ImGui::EndDisabled();
      ImGui::TextWrapped("%s", weather_status.c_str());
      if (weather_conditions) {
        draw_label_value("Cloud cover",
                         fixed(weather_conditions->cloud_cover_percent, 0) + "%");
        draw_label_value("Air temperature",
                         fixed(weather_conditions->air_temperature_c, 1) + " C");
        draw_label_value("Geographic sun azimuth",
                         fixed(weather_conditions->sun.azimuth_degrees, 1) +
                             " deg");
      }

      ImGui::Spacing();
      ImGui::TextDisabled("SUN & SKY");
      inputs_dirty |= ImGui::SliderFloat("Car-relative azimuth", &azimuth,
                                         0.0f, 360.0f, "%.1f deg");
      inputs_dirty |= ImGui::SliderFloat("Elevation", &elevation, -90.0f, 90.0f,
                                         "%.1f deg");
      inputs_dirty |=
          ImGui::SliderFloat("DNI", &sun.dni_w_m2, 0.0f, 1200.0f, "%.0f W/m2");
      inputs_dirty |=
          ImGui::SliderFloat("DHI", &sun.dhi_w_m2, 0.0f, 600.0f, "%.0f W/m2");
      sun.direction_to_sun =
          sun_direction_from_azimuth_elevation(azimuth, elevation);

      ImGui::Spacing();
      ImGui::TextDisabled("RAY TRACING");
      if (ImGui::SliderInt("Direct grid axis", &direct_samples, 1, 7)) {
        direct_samples = std::max(1, direct_samples);
        inputs_dirty = true;
      }
      if (ImGui::IsItemHovered())
        ImGui::SetTooltip("N x N direct-shadow samples across each cell");
      if (ImGui::SliderInt("Sky rays / cell", &ray_count, 8, 512)) {
        ray_count = std::max(8, ray_count);
        inputs_dirty = true;
      }
      if (ImGui::SliderInt("Reflection depth", &reflection_depth, 0, 4))
        inputs_dirty = true;
      settings.direct_samples_per_axis =
          static_cast<std::uint32_t>(direct_samples);
      settings.hemisphere_samples = static_cast<std::uint32_t>(ray_count);
      settings.max_reflection_depth =
          static_cast<std::uint32_t>(reflection_depth);

      if (inputs_dirty)
        ImGui::TextColored({0.98f, 0.67f, 0.20f, 1.0f},
                           "Inputs changed - run to refresh cell results");
      else
        ImGui::TextColored({0.35f, 0.80f, 0.55f, 1.0f}, "%s", status.c_str());

      ImGui::BeginDisabled(race_running);
      if (ImGui::Button("RUN SIMULATION", {-1.0f, 42.0f})) {
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
        status = "Completed in " + fixed(static_cast<float>(seconds), 2) + " s";
        inputs_dirty = false;
        race_average = false;
      }
      ImGui::EndDisabled();
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
        draw_label_value("Live shell preview",
                         fixed(*hovered_shell_irradiance, 1) + " W/m2");
      } else {
        ImGui::TextDisabled("Hover over the shell for live preview W/m2");
      }

      ImGui::Spacing();
      ImGui::TextDisabled("VIEW");
      ImGui::Checkbox("Shell", &show_shell);
      ImGui::SameLine(112.0f);
      ImGui::Checkbox("Grid", &show_grid);
      ImGui::SameLine(220.0f);
      ImGui::Checkbox("Wireframe", &wireframe);
      ImGui::Checkbox("Live shell heat", &show_shell_heat);
      if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Orientation-only preview; no shadow or reflection rays. Cell irradiance fill shows traced results.");
      ImGui::Checkbox("Generated layout", &show_cells);
      ImGui::SameLine(180.0f);
      ImGui::Checkbox("Sun direction", &show_sun);
      if (show_cells) {
        ImGui::Checkbox("Cell irradiance fill", &show_cell_heat);
        ImGui::TextColored({0.10f, 0.78f, 0.96f, 1.0f}, "2x3 outline");
        ImGui::SameLine(150.0f);
        ImGui::TextColored({1.00f, 0.55f, 0.12f, 1.0f}, "1x3 outline");
      }
      if (ImGui::Button("Reset camera", {-1.0f, 0.0f})) camera.reset();
      if (track_view && ImGui::Button("Reset track camera", {-1.0f, 0.0f})) track_camera.reset();

      ImGui::Spacing();
      ImGui::Separator();
      ImGui::TextDisabled("%zu triangles  |  drag to orbit  |  wheel to zoom",
                          mesh.triangle_count());
      ImGui::TextDisabled("Electrical estimate: constant efficiency, no MPPT/string losses");
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
      const int track_pane_width = track_view ? std::max(1, viewport_width / 2) : 0;
      const int car_pane_x = viewport_x + track_pane_width;
      const int car_pane_width = std::max(1, viewport_width - track_pane_width);

      glViewport(0, 0, framebuffer_width, framebuffer_height);
      glClearColor(0.012f, 0.023f, 0.036f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      glViewport(car_pane_x, 0, car_pane_width, framebuffer_height);

      const float cp = std::cos(camera.pitch);
      const glm::vec3 eye = as_glm(camera.target) +
                            camera.distance * glm::vec3(
                                cp * std::cos(camera.yaw),
                                cp * std::sin(camera.yaw),
                                std::sin(camera.pitch));
      const glm::mat4 view = glm::lookAt(
          eye, as_glm(camera.target), glm::vec3(0.0f, 0.0f, 1.0f));
      const glm::mat4 projection = glm::perspective(
          glm::radians(43.0f), static_cast<float>(car_pane_width) /
                                  static_cast<float>(std::max(1, framebuffer_height)),
          std::max(0.01f, camera.home_distance * 0.0001f),
          std::max(250.0f, camera.home_distance * 20.0f));
      const glm::mat4 vp = projection * view;

      hovered_shell_irradiance.reset();
      double cursor_x = 0.0, cursor_y = 0.0;
      glfwGetCursorPos(window, &cursor_x, &cursor_y);
      const double viewport_window_width =
          (static_cast<double>(window_width) - kPanelWidth) * (track_view ? 0.5 : 1.0);
      const double car_window_x = kPanelWidth + (track_view ? viewport_window_width : 0.0);
      if (!io.WantCaptureMouse && cursor_x >= car_window_x &&
          cursor_x < window_width && cursor_y >= 0.0 &&
          cursor_y < window_height && viewport_window_width > 1.0) {
        const float ndc_x = static_cast<float>(
            2.0 * (cursor_x - car_window_x) / viewport_window_width - 1.0);
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
        if (hit) {
          Vec3 normal = hit->normal;
          if (normal.z < 0.0f) normal = -normal;
          const float direct =
              sun.dni_w_m2 *
              std::max(0.0f, dot(normal, sun.direction_to_sun));
          const float sky =
              sun.dhi_w_m2 *
              std::clamp((1.0f + normal.z) * 0.5f, 0.0f, 1.0f);
          hovered_shell_irradiance = direct + sky;
        }
      }

      glUseProgram(program);
      glUniformMatrix4fv(glGetUniformLocation(program, "uViewProjection"), 1,
                         GL_FALSE, glm::value_ptr(vp));
      glUniform3f(glGetUniformLocation(program, "uSunDirection"),
                  sun.direction_to_sun.x, sun.direction_to_sun.y,
                  sun.direction_to_sun.z);
      glUniform1f(glGetUniformLocation(program, "uDni"), sun.dni_w_m2);
      glUniform1f(glGetUniformLocation(program, "uDhi"), sun.dhi_w_m2);

      glUniform1i(glGetUniformLocation(program, "uUnlit"), GL_TRUE);
      glUniform1i(glGetUniformLocation(program, "uCellPass"), GL_FALSE);
      glUniform1i(glGetUniformLocation(program, "uShellHeat"), GL_FALSE);
      if (track_view && !rendered_track.empty()) {
        glViewport(viewport_x, 0, track_pane_width, framebuffer_height);
        const float cp_track = std::cos(track_camera.pitch);
        const auto eye_track = as_glm(track_camera.target) + track_camera.distance * glm::vec3(
            cp_track * std::cos(track_camera.yaw), cp_track * std::sin(track_camera.yaw), std::sin(track_camera.pitch));
        const auto track_vp = glm::perspective(glm::radians(43.0f),
            static_cast<float>(track_pane_width) / std::max(1, framebuffer_height),
            std::max(0.01f, track_camera.home_distance * 0.0001f),
            std::max(250.0f, track_camera.home_distance * 20.0f)) *
            glm::lookAt(eye_track, as_glm(track_camera.target), glm::vec3(0, 0, 1));
        glUniformMatrix4fv(glGetUniformLocation(program, "uViewProjection"), 1,
                           GL_FALSE, glm::value_ptr(track_vp));
        glBindVertexArray(track_vao);
        glLineWidth(2.0f);
        glDrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(rendered_track.size()));
        glViewport(car_pane_x, 0, car_pane_width, framebuffer_height);
        glUniformMatrix4fv(glGetUniformLocation(program, "uViewProjection"), 1,
                           GL_FALSE, glm::value_ptr(vp));
      }
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

      if (!rendered_top.empty()) {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(-1.0f, -1.0f);
        glUniform1i(glGetUniformLocation(program, "uUnlit"), GL_FALSE);
        glUniform1i(glGetUniformLocation(program, "uShellHeat"), GL_FALSE);
        glUniform1i(glGetUniformLocation(program, "uCellPass"), GL_FALSE);
        glBindVertexArray(top_vao);
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(rendered_top.size()));
        glDisable(GL_POLYGON_OFFSET_FILL);
      }

      if (show_cells) {
        if (show_cell_heat) {
          glEnable(GL_POLYGON_OFFSET_FILL);
          glPolygonOffset(-1.0f, -1.0f);
          glUniform1i(glGetUniformLocation(program, "uUnlit"), GL_FALSE);
          glUniform1i(glGetUniformLocation(program, "uCellPass"), GL_TRUE);
          glUniform1i(glGetUniformLocation(program, "uShellHeat"), GL_FALSE);
          glBindVertexArray(cells_vao);
          glDrawArrays(GL_TRIANGLES, 0,
                       static_cast<GLsizei>(rendered_cells.size()));
          glDisable(GL_POLYGON_OFFSET_FILL);
        }
        glUniform1i(glGetUniformLocation(program, "uUnlit"), GL_TRUE);
        glBindVertexArray(outlines_vao);
        glLineWidth(2.0f);
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
    }
    if (imgui_glfw_initialized) ImGui_ImplGlfw_Shutdown();
    if (imgui_context_created) ImGui::DestroyContext();
    if (program) glDeleteProgram(program);
    for (GLuint buffer : {shell_vbo, cells_vbo, outlines_vbo, grid_vbo, sun_vbo, track_vbo, top_vbo})
      if (buffer) glDeleteBuffers(1, &buffer);
    for (GLuint array : {shell_vao, cells_vao, outlines_vao, grid_vao, sun_vao, track_vao, top_vao})
      if (array) glDeleteVertexArrays(1, &array);
    glfwDestroyWindow(window);
    glfwTerminate();
    throw;
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glDeleteProgram(program);
  for (GLuint buffer : {shell_vbo, cells_vbo, outlines_vbo, grid_vbo, sun_vbo, track_vbo, top_vbo})
    glDeleteBuffers(1, &buffer);
  for (GLuint array : {shell_vao, cells_vao, outlines_vao, grid_vao, sun_vao, track_vao, top_vao})
    glDeleteVertexArrays(1, &array);
  glfwDestroyWindow(window);
  glfwTerminate();
}

}  // namespace solar
