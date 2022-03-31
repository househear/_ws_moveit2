// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Description: This controller demonstrates how to use openCV 2.4 to process the camera image.
 *              In order to execute and recompile this example, opencv must be installed.
 *              To run this controller, it is recommended to install the Webots development
 *              environment as explained here:
 * https://github.com/cyberbotics/webots/wiki#installation-of-the-webots-development-environment
 *
 */
#include <webots/supervisor.h>
#include <stdio.h>
#include <webots/camera.h>
#include <webots/display.h>
#include <webots/keyboard.h>
#include <webots/robot.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>
#include <unistd.h>
#define NB_FILTERS 6

#define RED 0
#define GREEN 1
#define BLUE 2
#define YELLOW 3
#define PURPLE 4
#define WHITE 5
#define NONE 6
#define ALL 7



#include <math.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#ifdef _WIN32
#include <winsock.h>
#else
#include <arpa/inet.h>  /* definition of inet_ntoa */
#include <netdb.h>      /* definition of gethostbyname */
#include <netinet/in.h> /* definition of struct sockaddr_in */
#include <sys/socket.h>
#include <sys/time.h>
#endif

#define SOCKET_PORT 10020
#define NB_IR_SENSOR 8
#define TIMESTEP 250




using namespace cv;
using namespace std;

/* The scalars correspond to HSV margin (In the first example, [0,5] is the accepted hue for the red filter,
   [150,255] the accepted saturation and [30,255] the accepted value). */
static Scalar lMargin[NB_FILTERS] = {Scalar(0, 150, 30),  Scalar(58, 150, 30),  Scalar(115, 150, 30),
                                     Scalar(28, 150, 30), Scalar(148, 150, 30), Scalar(0, 0, 50)};
static Scalar uMargin[NB_FILTERS] = {Scalar(5, 255, 255),  Scalar(62, 255, 255),  Scalar(120, 255, 255),
                                     Scalar(32, 255, 255), Scalar(152, 255, 255), Scalar(0, 0, 255)};

/* we use the last spot to quickly check if filters are used or not. */
static bool filters[NB_FILTERS + 1] = {false, false, false, false, false, false, true};
static int width;
static int height;
static unsigned char *processed_image;
static int fd;
static fd_set rfds;



  double fix[3] = {1.71, 1.31, 5.63};
  double color_Current_P[3] = {0, 1, 0};
  double color_Waiting_P[3] = {1, 0, 0};
  double color_Reached_P[3] = {0, 0, 1};


static int accept_client(int server_fd) {
  int cfd;
  struct sockaddr_in client;
#ifndef _WIN32
  socklen_t asize;
#else
  int asize;
#endif
  struct hostent *client_info;

  asize = sizeof(struct sockaddr_in);

  cfd = accept(server_fd, (struct sockaddr *)&client, &asize);
  if (cfd == -1) {
    printf("cannot accept client\n");
    return -1;
  }
  client_info = gethostbyname((char *)inet_ntoa(client.sin_addr));
  printf("Accepted connection from: %s \n", client_info->h_name);

  return cfd;
}

static int create_socket_server(int port) {
  int sfd, rc;
  struct sockaddr_in address;

#ifdef _WIN32
  /* initialize the socket api */
  WSADATA info;

  rc = WSAStartup(MAKEWORD(1, 1), &info); /* Winsock 1.1 */
  if (rc != 0) {
    printf("cannot initialize Winsock\n");
    return -1;
  }
#endif
  /* create the socket */
  sfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sfd == -1) {
    printf("cannot create socket\n");
    return -1;
  }

  /* fill in socket address */
  memset(&address, 0, sizeof(struct sockaddr_in));
  address.sin_family = AF_INET;
  address.sin_port = htons((unsigned short)port);
  address.sin_addr.s_addr = INADDR_ANY;

  /* bind to port */
  rc = bind(sfd, (struct sockaddr *)&address, sizeof(struct sockaddr));
  if (rc == -1) {
    printf("cannot bind port %d\n", port);
#ifdef _WIN32
    closesocket(sfd);
#else
    close(sfd);
#endif
    return -1;
  }

  /* listen for connections */
  if (listen(sfd, 1) == -1) {
    printf("cannot listen for connections\n");
#ifdef _WIN32
    closesocket(sfd);
#else
    close(sfd);
#endif
    return -1;
  }
  printf("Waiting for a connection on port %d...\n", port);

  return accept_client(sfd);
}



void display_commands() {
  printf("Press R to apply/remove a red filter.\n");
  printf("Press G to apply/remove a green filter.\n");
  printf("Press B to apply/remove a blue filter.\n");
  printf("Press Y to apply/remove a yellow filter.\n");
  printf("Press P to apply/remove a purple filter.\n");
  printf("Press W to apply/remove a white filter.\n");
  printf("Press A to apply all filters.\n");
  printf("Press X to remove all filters.\n");
  printf("When one or several filter is applied, only the corresponding colors are considered in the image.\n");
  printf("The processed image consists of the entire image if no filter is used.\n");
}

/* Function to process the image from the camera and display the result.
   The only processing this function does is only displaying parts of the image which
   correspond to one of the predefined filters. */
void process_image(const unsigned char *image, int length) {
  /* Matrix which contains the BGRA image from Webots' camera */
  Mat img = Mat(Size(width, height), CV_8UC4);
  img.data = (uchar *)image;

  /* Matrix which contains the HSV version of the previous image */
  Mat hsv = Mat(Size(width, height), CV_8UC3);
  cvtColor(img, hsv, COLOR_BGR2HSV);

  /* Temporary data corresponding to the HSV image filtered through one filter */
  Mat temp_filtered = Mat(Size(width, height), CV_8UC1);

  /* Matrix which will contain the post-processing image */
  Mat filtered = Mat(Size(width, height), CV_8UC4);

  if (filters[NB_FILTERS])
    filtered = img;
  else {
    /* Initialize the output matrix in the case we have to build it */
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        filtered.at<Vec4b>(i, j)[0] = 0;
        filtered.at<Vec4b>(i, j)[1] = 0;
        filtered.at<Vec4b>(i, j)[2] = 0;
        filtered.at<Vec4b>(i, j)[3] = 255;
      }
    }

    for (int f = 0; f < NB_FILTERS; ++f) {
      if (filters[f]) {
        inRange(hsv, lMargin[f], uMargin[f], temp_filtered);
        /* Copy the value from the original image to the output if it's accepted by a filter */
        for (int i = 0; i < height; ++i) {
          for (int j = 0; j < width; ++j) {
            if (temp_filtered.at<uchar>(i, j) == 255) {
              filtered.at<Vec4b>(i, j) = img.at<Vec4b>(i, j);
            }
          }
        }
      }
    }
  }

  /* Refresh the picture to display */
  memcpy(processed_image, filtered.data, length);
}

/* This function handles which filters need to be used when the display function is called. */
void apply_filter(int filter) {

    printf("capturing a image and calculating the lable 3D pose.\n");

}









static void create_trail_shape(int& P_index, const double* target_translation, const double* target_color, char _group_name[]) {
  // If TRAIL exists in the world then silently remove it.

  char trail_string[0x10000] = "\0";  // Initialize a big string which will contain the TRAIL node.

  char buffer[10];
  snprintf(buffer, 10, "%d", P_index);

  char group_name[20] = {};
  char point_name[20] = {};


  strcat(group_name, _group_name);

  strcat(point_name, group_name);
  strcat(point_name, buffer);




  strcat(trail_string, "  #VRML_OBJ R2021a utf8\n");
  strcat(trail_string, "DEF ");
  strcat(trail_string, point_name);
  strcat(trail_string, " KukaBox {\n");
  strcat(trail_string, "  translation 1.71 1.31 5.63\n");
  strcat(trail_string, "  rotation 1 0 0 0\n");
  strcat(trail_string, "}\n");

  // Import TRAIL and append it as the world root nodes.
  WbFieldRef Current_P_children_field = wb_supervisor_node_get_field(wb_supervisor_node_get_from_def(group_name), "children");
  wb_supervisor_field_import_mf_node_from_string(Current_P_children_field, -1, trail_string);
  printf("%s\n", point_name);
   //get the CP point just added;
  WbFieldRef Current_P_children_CP_trans_field = wb_supervisor_node_get_field(wb_supervisor_node_get_from_def(point_name), "translation");
  WbFieldRef Current_P_children_CP_color_field = wb_supervisor_node_get_field(wb_supervisor_node_get_from_def(point_name), "color");
      // Add the new target translation in the line set.
 // printf("bb\n");
  wb_supervisor_field_set_sf_vec3f(Current_P_children_CP_trans_field, target_translation);
  wb_supervisor_field_set_sf_color(Current_P_children_CP_color_field, target_color);
  P_index = P_index + 1;
}




static void run(int& current_P_index, int& waiting_P_index, int& reached_P_index) {
  int n;
  int ret;
  char buffer[256];
  int left_speed, right_speed;
  short led_number, led_action;
  static short led_value[2] = {0, 0}; /* initially off */
  struct timeval tv = {0, 0};
  int number;


  /* Set up the parameters used for the select statement */

  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);

  /*
   * Watch TCPIP file descriptor to see when it has input.
   * No wait - polling as fast as possible
   */
  number = select(fd + 1, &rfds, NULL, NULL, &tv);

  /* If there is no data at the socket, then redo loop */
  if (number == 0)
    return;

  /* ...otherwise, there is data to read, so read & process. */
  n = recv(fd, buffer, 256, 0);
  if (n < 0) {
    printf("error reading from socket\n");
    return;
  }
  buffer[n] = '\0';
  printf("Received %d bytes: %s\n", n, buffer);

  if (buffer[0] == 'D') { /* set the speed of the motors */
    sscanf(buffer, "D,%d,%d", &left_speed, &right_speed);

    send(fd, "d\r\n", 3, 0);

  } else if (buffer[0] == 'L') {
    sscanf(buffer, "L,%hd,%hd", &led_number, &led_action);
    if (led_number >= 0 && led_number <= 1 && led_action >= 0 && led_action <= 2) {
      if (led_action == 2) { /* change status */
        if (led_value[led_number] == 1)
          led_action = 0; /* switch off */
        else
          led_action = 1; /* switch on */
      }
      led_value[led_number] = led_action;
          fix[0] = 1.71 + 0.03 * current_P_index;
          fix[1] = 1.31;
          create_trail_shape(current_P_index, fix, color_Current_P, (char *)"Current_P");
      printf("set led %d to %d\n", led_number, led_action);

    }
    send(fd, "l\r\n", 3, 0);

  } else if (buffer[0] == 'G') { /* set the position counter */
    int left, right;
    sscanf(buffer, "G,%d,%d", &left, &right);
              fix[0] = 1.71 + 0.03 * waiting_P_index;
          fix[1] = 1.31 + 0.05;
          create_trail_shape(waiting_P_index, fix, color_Waiting_P, (char *)"Waiting_P");
    send(fd, "g\r\n", 3, 0);

  } else if (buffer[0] == 'B') { /* return a pretend version string */
    sprintf(buffer, "b,0,0\r\n");
              fix[0] = 1.71 + 0.03 * reached_P_index;
          fix[1] = 1.31 + 0.10;
          create_trail_shape(reached_P_index, fix, color_Reached_P, (char *)"Reached_P");
  
    send(fd, buffer, strlen(buffer), 0);

  } else if (strncmp(buffer, "exit", 4) == 0) {
    printf("connection closed\n");
#ifdef _WIN32
    closesocket(fd);
    ret = WSACleanup();
#else
    ret = close(fd);
#endif
    if (ret != 0) {
      printf("Cannot close socket\n");
    }
    fd = 0;
  } else {
    send(fd, "\n", 1, 0);
  }
}




int main() {
  /* Initialize Webots */
  wb_robot_init();
  
  
  
  
  printf("init tcp\n");
  fd = create_socket_server(SOCKET_PORT);
  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);
  
  
  

/* import tracking point 8*/
  
  int timestep = wb_robot_get_basic_time_step();

  printf("Vision module demo, using openCV.\n");

  display_commands();




  /* Initialize real camera and open capture */
  cout << "Built with OpenCV " << CV_VERSION << endl;
  Mat captured_image;
  VideoCapture capture;
  capture.open(0);
  if(capture.isOpened())
  {
      cout << "YD: Capture is opened..." << endl;
  }
  else
  {
      cout << "No capture" << endl;
  }


  /* Initialize camera */
  //WbDeviceTag camera = wb_robot_get_device("camera");
  //wb_camera_enable(camera, timestep);
  width = 640;//wb_camera_get_width(camera);
  height = 480;//wb_camera_get_height(camera);

  /* Variables for the display */
  int length = 4 * width * height * sizeof(unsigned char);
  WbDeviceTag processed_image_display = wb_robot_get_device("proc_im_display");
  WbImageRef processed_image_ref = NULL;
  processed_image = (unsigned char *)malloc(length);

  wb_keyboard_enable(timestep);

  int input = 0;
  bool key_pressed = false;


  int current_P_index = 0;
  int Waiting_P_index = 0;
  int Reached_P_index = 0;

  while (wb_robot_step(timestep) != -1) {
  run(current_P_index, Waiting_P_index, Reached_P_index);
    usleep(50*1000);
    /* Process inputs */
    const int key = wb_keyboard_get_key();
    if (key >= 0 && !key_pressed) {
      key_pressed = true;
      input = key;
    } else if (key == -1 && key_pressed) {
      key_pressed = false;
      switch (input) {
        case 'C':
          fix[0] = 1.71 + 0.03 * current_P_index;
          fix[1] = 1.31;
          create_trail_shape(current_P_index, fix, color_Current_P, (char *)"Current_P");
          //current_P_index = current_P_index + 1;
          break;
        case 'W':
          fix[0] = 1.71 + 0.03 * Waiting_P_index;
          fix[1] = 1.31 + 0.05;
          create_trail_shape(Waiting_P_index, fix, color_Waiting_P, (char *)"Waiting_P");
          //Waiting_P_index = Waiting_P_index + 1;
          break;
        case 'R':
          fix[0] = 1.71 + 0.03 * Reached_P_index;
          fix[1] = 1.31 + 0.10;
          create_trail_shape(Reached_P_index, fix, color_Reached_P, (char *)"Reached_P");
          //Reached_P_index = Reached_P_index + 1;
          break;
        case 'B':
          apply_filter(BLUE);
          break;
        case 'Y':
          apply_filter(YELLOW);
          break;
        case 'P':
          apply_filter(PURPLE);
          break;
        case 'A':
          apply_filter(ALL);
          break;
        default:
          break;
      }
    }


    capture >> captured_image;



    /* Process the image */
    memcpy(processed_image, captured_image.data, 3*480*640);
    //process_image(wb_camera_get_image(camera), length);

    if (processed_image_ref) {
      wb_display_image_delete(processed_image_display, processed_image_ref);
      processed_image_ref = NULL;
    }

    /* Display the image */
    processed_image_ref = wb_display_image_new(processed_image_display, width, height, processed_image, WB_IMAGE_RGB);
     //processed_image_ref = wb_display_image_load(processed_image_display, "test.png");

    wb_display_image_paste(processed_image_display, processed_image_ref, 0, 0, false);
  }

  // clean up
  if (processed_image_ref)
    wb_display_image_delete(processed_image_display, processed_image_ref);
  free(processed_image);

  wb_robot_cleanup();

  return 0;
}
