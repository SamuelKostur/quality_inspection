  #include <ros/ros.h>
  #include <quality_inspection/MovRobToScanPosAction.h>
  #include <actionlib/server/simple_action_server.h>
  #include <stdlib.h>
  #include <tinyxml.h>
  #include <stdio.h>
  #include <string.h>

  // Socket
  #include <sys/types.h>
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <errno.h>

  typedef actionlib::SimpleActionServer<quality_inspection::MovRobToScanPosAction> Server;

  class RobotPoseSender{
    public: 
      ros::NodeHandle n;
      actionlib::SimpleActionServer<quality_inspection::MovRobToScanPosAction> as;
      int sockfd_client;

      RobotPoseSender() : as(n, "movRobToScanPos", boost::bind(&RobotPoseSender::executeCB, this, _1), false){
        if(initSocket()!=0){
          std::cout << "Connection to robot unsuccessful" << std::endl;
          return;
        }
        as.start();

        ros::spin();
      }

      int sendRobotCommand(double robot_pose_command [6]){
        TiXmlDocument xml_out;
        TiXmlElement* robot_command = new TiXmlElement("RobotCommand");
        TiXmlElement* pose = new TiXmlElement("Pose");
        TiXmlText* empty_text = new TiXmlText("");

        robot_command->LinkEndChild(pose);
        pose->LinkEndChild(empty_text);   // force <Position></Position> format (vs <Position />)

        pose->SetAttribute("X", std::to_string(robot_pose_command[0]).c_str());
        pose->SetAttribute("Y", std::to_string(robot_pose_command[1]).c_str());
        pose->SetAttribute("Z", std::to_string(robot_pose_command[2]).c_str());
        pose->SetAttribute("A", std::to_string(robot_pose_command[3]).c_str());
        pose->SetAttribute("B", std::to_string(robot_pose_command[4]).c_str());
        pose->SetAttribute("C", std::to_string(robot_pose_command[5]).c_str());
        xml_out.LinkEndChild(robot_command);
        
        TiXmlPrinter xml_printer;
        xml_printer.SetStreamPrinting();  // no linebreaks
        xml_out.Accept(&xml_printer);
        xml_out.SaveFile("pozicieRobota.xml");  

        double sentStrLen = send(sockfd_client ,xml_printer.CStr(), xml_printer.Size(), 0);
        printf("Send vratil = %lf \n", sentStrLen);

        if(sentStrLen > 0)
          return 0;
        else{
          printf("Error while sending desired position \n");
          return -1;
        }
      }

      int readRobotPose(quality_inspection::MovRobToScanPosResult *robotPos){
        char readBuffer[1024] = {0};
        //int desReadStrLen = ; //error checking
        TiXmlDocument xml_in;
        int readStrLen = read(sockfd_client, readBuffer, 1024);
        xml_in.Parse(readBuffer);
        xml_in.FirstChildElement("RobotState") -> FirstChildElement("Position") -> QueryDoubleAttribute("X", &robotPos->x);
        xml_in.FirstChildElement("RobotState") -> FirstChildElement("Position") -> QueryDoubleAttribute("Y", &robotPos->y);
        xml_in.FirstChildElement("RobotState") -> FirstChildElement("Position") -> QueryDoubleAttribute("Z", &robotPos->z);
        xml_in.FirstChildElement("RobotState") -> FirstChildElement("Orientation") -> QueryDoubleAttribute("A", &robotPos->A);
        xml_in.FirstChildElement("RobotState") -> FirstChildElement("Orientation") -> QueryDoubleAttribute("B", &robotPos->B);
        xml_in.FirstChildElement("RobotState") -> FirstChildElement("Orientation") -> QueryDoubleAttribute("C", &robotPos->C);
        memset(readBuffer, 0, sizeof(readBuffer));
        xml_in.Clear();
        return 0;
      }

      void executeCB(const quality_inspection::MovRobToScanPosGoalConstPtr& goal){
        double robot_pose_command[6];  
        robot_pose_command[0] = (double) goal->x;
        robot_pose_command[1] = (double) goal->y;
        robot_pose_command[2] = (double) goal->z;
        robot_pose_command[3] = (double) goal->A;
        robot_pose_command[4] = (double) goal->B;
        robot_pose_command[5] = (double) goal->C;    
        if(sendRobotCommand(robot_pose_command) != 0){
          as.setAborted();
        }
        quality_inspection::MovRobToScanPosResult robotPos;

        readRobotPose(&robotPos); //for real robot
        // sleep(3); //for testing on PC
        // robotPos.x = goal->x; //for testing on PC

        // Do lots of awesome groundbreaking robot stuff here
        as.setSucceeded(robotPos);
      }

      int initSocket(){
        // Create socket
        int sockfd_server = socket(AF_INET, SOCK_STREAM, 0);
        if(sockfd_server < 0)
        {
          printf("Not able to create socket! \n");
          return -1;
        }

        // Allow reuse of local addresses and check return value
        int option = 1;
        if (setsockopt(sockfd_server, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option)) != 0)
        {
          shutdown(sockfd_server, SHUT_RDWR);
          close(sockfd_server);
          printf("Server: setsockopt() error \n");
          return -1;
        }

        // Assign address to the socket
        struct sockaddr_in server_addr;
        server_addr.sin_addr.s_addr = inet_addr("192.168.1.2"); //192.168.1.2, 127.0.0.1
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(54603);
        if (bind(sockfd_server, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0)
        {
          printf("ERROR on binding socket: %s\n", strerror(errno));
          return -1;
        }

        // Enable accepting connections
        listen(sockfd_server, 1);

        printf("Waiting for connection from KUKA controller...\n");

        struct sockaddr_in client_addr;
        socklen_t client_length = sizeof(client_addr);

        sockfd_client = accept(sockfd_server, (struct sockaddr *) &client_addr, &client_length);
        if (sockfd_client < 0)
        {
          printf("ERROR on accept: %s\n", strerror(errno));
          return -1;
        }
    
        printf("Client on robot connected \n");
        return 0;
      }
  };
  
  int main(int argc, char** argv){
    ros::init(argc, argv, "movRobToScanPosServer");
    RobotPoseSender robotPoseSender;
    return 0;
  }