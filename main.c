#define GLFW_INCLUDE_GLU

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <GLFW/glfw3.h>

double sample_rates[] = { 1000000.0, 250000.0, 44100.0, 1.0 / (0.054611 / 8192.0) };
unsigned current_sample_rate = 0;
unsigned current_error = 0;

float scale = 1.f;

void print_settings()
{
  printf("Current settings\n");
  printf("================\n");
  printf("Sample rate: %fHz\n", sample_rates[current_sample_rate]);
  printf("Error: +/-%d samples\n\n", current_error);
}

void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
  if (action == GLFW_PRESS) {
    if (key == GLFW_KEY_UP) {
      ++current_error;
      print_settings();
    }
    if (key == GLFW_KEY_DOWN && current_error > 0) {
      --current_error;
      print_settings();
    }
    if (key == GLFW_KEY_RIGHT && current_sample_rate < 3) {
      ++current_sample_rate;
      print_settings();
    }
    if (key == GLFW_KEY_LEFT && current_sample_rate > 0) {
      --current_sample_rate;
      print_settings();
    }
  }
}

void scroll_callback(GLFWwindow *window, double x, double y) {
  scale += y * 0.1;
  scale = fmax(scale, 0.0);
}

int main(void)
{
  if (!glfwInit()) {
    return EXIT_FAILURE;
  }

  glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
  GLFWwindow *window = glfwCreateWindow(1280, 720, "BIR Simulation", NULL, NULL);
  if (!window) {
    glfwTerminate();
    return -1;
  }

  glfwMakeContextCurrent(window);
  glfwSetKeyCallback(window, key_callback);
  glfwSetScrollCallback(window, scroll_callback);
  //glClearColor(1.0, 1.0, 1.0, 0.0);

  print_settings();

  float aspect = 1280.f / 720.f;
  double mic_radius = 0.17f / 2.f;
  double c = 3300.0;

  //1.5us per sample

  glMatrixMode(GL_PROJECTION);
  glOrtho(-aspect, aspect, -1.f, 1.f, -1.f, 1.f);

  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  double mics[4][2] = {
    { mic_radius, 0.0 },
    { 0.0, mic_radius },
    { -mic_radius, 0.0 },
    { 0.0, -mic_radius }
  };

  double prev_points[60][2];
  int index = 0;

  while (!glfwWindowShouldClose(window)) {
    glClear(GL_COLOR_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glScalef(scale, scale, scale);

    glPointSize(10.f);
    glLineWidth(2.f);

    glColor3f(1.f, 1.f, 1.f);
    glBegin(GL_POINTS);
    for (int i = 0; i < 4; ++i) {
      glVertex2f(mics[i][0], mics[i][1]);
    }
    glEnd();

    double x, y;
    glfwGetCursorPos(window, &x, &y);

    GLint viewport[4] = { 0, 0, 1280, 720 };
    //glGetIntegerv(GL_VIEWPORT, viewport);
    GLdouble modelview[16];
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    GLdouble projection[16];
    glGetDoublev(GL_PROJECTION_MATRIX, projection);

    y = viewport[3] - y;
    double posX, posY, posZ;
    gluUnProject(x, y, 0.f, modelview, projection, viewport, &posX, &posY, &posZ);

    double t[4];
    for (int i = 0; i < 4; ++i) {
      double sample_period = 1.0 / sample_rates[current_sample_rate];

      t[i] = sqrt(pow(mics[i][0] - posX, 2) + pow(mics[i][1] - posY, 2)) / c;

      t[i] = floor(t[i] / sample_period) * sample_period;

      t[i] += (rand() / (double) RAND_MAX - 0.5) * 2.0 * sample_period * current_error;
    }

    double phi;

    if (fabs(t[3] - t[1]) >= fabs(t[2] - t[0])) {
      phi = atan2(t[3] - t[1], t[2] - t[0]);
    }
    else {
      phi = atan2(t[2] - t[0], t[1] - t[3]) - M_PI / 2;
    }

    double r = c * (t[0] + t[1] + t[2] + t[3]) / 4;
    prev_points[index][0] = r * cos(phi);
    prev_points[index][1] = r * sin(phi);
    index = (index + 1) % 60;

    for (int i = index + 1, j = 0; i != index; i = (i + 1) % 60, ++j) {
      glColor4f(1.f, 0.f, 0.f, j / 60.f);
      glBegin(GL_LINES);
        glVertex2d(0.0, 0.0);
        glVertex2d(prev_points[i][0], prev_points[i][1]);
      glEnd();
      glColor4f(0.f, 0.f, 1.f, j / 60.f);
      glBegin(GL_POINTS);
      glVertex2d(prev_points[i][0], prev_points[i][1]);
      glEnd();
    }

    glColor3f(0.f, 1.f, 0.f);
    double avg_x = 0.0, avg_y = 0.0;
    for (int i = 0; i < 60; ++i) {
      avg_x += prev_points[i][0];
      avg_y += prev_points[i][1];
    }
    avg_x /= 60; avg_y /= 60;
    glBegin(GL_POINTS);
      glVertex2f(avg_x, avg_y);
    glEnd();

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  glfwTerminate();

  return EXIT_SUCCESS;
}
