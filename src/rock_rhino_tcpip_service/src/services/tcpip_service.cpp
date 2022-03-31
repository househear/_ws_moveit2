// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#include <cinttypes>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "example_interfaces/srv/add_two_ints.hpp"

#include "rock_rhino_tcpip_service/visibility_control.h"



#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#include <winsock.h>
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#define SOCKET_PORT 10020
#define SOCKET_SERVER "127.0.0.1" /* local host */



namespace rock_rhino_tcpip_service
{

class ServerNode : public rclcpp::Node
{
public:
  ROCK_RHINO_TCPIP_SERVICE_PUBLIC
  explicit ServerNode(const rclcpp::NodeOptions & options)
  : Node("tcpip_service", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    i = 100;
    auto handle_connect_tcpip =
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) -> void
      {
        (void)request_header;
          #ifdef _WIN32
            /* initialize the socket api */
            WSADATA info;

            rc = WSAStartup(MAKEWORD(1, 1), &info); /* Winsock 1.1 */
            if (rc != 0) {
              printf("cannot initialize Winsock\n");

            }
          #endif
            /* create the socket */
            fd = socket(AF_INET, SOCK_STREAM, 0);
            if (fd == -1) {
               RCLCPP_INFO(
          this->get_logger(), "cannot create socket\n");

              printf("cannot create socket\n");
            }
            RCLCPP_INFO(
          this->get_logger(), "created socket\n");

            /* fill in the socket address */
            memset(&address, 0, sizeof(struct sockaddr_in));
            address.sin_family = AF_INET;
            address.sin_port = htons(SOCKET_PORT);
            server = gethostbyname(SOCKET_SERVER);

            if (server)
              memcpy((char *)&address.sin_addr.s_addr, (char *)server->h_addr, server->h_length);
            else {
              printf("cannot resolve server name: %s\n", SOCKET_SERVER);
          #ifdef _WIN32
              closesocket(fd);
          #else
              close(fd);
          #endif

            }

            /* connect to the server */
            rc = connect(fd, (struct sockaddr *)&address, sizeof(struct sockaddr));
            if (rc == -1) {
              printf("cannot connect to the server\n");
          #ifdef _WIN32
              closesocket(fd);
          #else
              close(fd);
          #endif

            }
          response->sum = fd;
      };








      auto handle_send_request =
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) -> void
      {
        (void)request_header;

        printf("Enter command: ");
        fflush(stdout);
        scanf("%255s", buffer);
        int n = strlen(buffer);
        buffer[n++] = '\n'; /* append carriage return */
        buffer[n] = '\0';
        n = send(fd, buffer, n, 0);
        n = recv(fd, buffer, 256, 0);
        buffer[n] = '\0';
        printf("Answer is: %s", buffer);

      };




      auto handle_close_tcpip =
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) -> void
      {
         (void)request_header;

        buffer[0] = 'e';
        buffer[1] = 'x';

        buffer[2] = 'i';
        buffer[3] = 't';

        int n = strlen(buffer);
        buffer[n++] = '\n'; /* append carriage return */
        buffer[n] = '\0';
        n = send(fd, buffer, n, 0);
        n = recv(fd, buffer, 256, 0);
        buffer[n] = '\0';
        printf("TCP/IP connection closed");

        #ifdef _WIN32
          closesocket(fd);
        #else
          close(fd);
        #endif

      };

      

 /*     std::function<void(const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request>,
  std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>)> fcn2;
fcn2 = std::bind(handle_add_two_ints, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, i);



    // Create a service that will use the callback function to handle requests.
    srv_ = create_service<example_interfaces::srv::AddTwoInts>("connect_tcpip", fcn2);*/
    srv_connect_tcpip = create_service<example_interfaces::srv::AddTwoInts>("tcpip/connect_tcpip", handle_connect_tcpip);
    srv_send_request = create_service<example_interfaces::srv::AddTwoInts>("tcpip/send_request", handle_send_request);
    srv_close_tcpip = create_service<example_interfaces::srv::AddTwoInts>("tcpip/close_tcpip", handle_close_tcpip);
  }

private:
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_connect_tcpip;
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_send_request;
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_close_tcpip;
  int i;


  struct sockaddr_in address;
  struct hostent *server;
  int fd, rc, fd1;
  char buffer[256];

};

}  // namespace rock_rhino_tcpip_service

RCLCPP_COMPONENTS_REGISTER_NODE(rock_rhino_tcpip_service::ServerNode)
