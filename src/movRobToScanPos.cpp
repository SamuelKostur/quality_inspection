  #include <ros/ros.h>
  #include <quality_inspection/MovRobToScanPosAction.h>
  #include <actionlib/server/simple_action_server.h>
  #include <stdlib.h>
  #include <tinyxml.h>

  // Socket
  #include <sys/types.h>
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <errno.h>
  #include <stdio.h>
  #include <string.h>

  typedef actionlib::SimpleActionServer<quality_inspection::MovRobToScanPosAction> Server;

  class RobotPoseSender{
    public: 
    ros::NodeHandle n;
    actionlib::SimpleActionServer<quality_inspection::MovRobToScanPosAction> as;
    int sockfd_client;

    RobotPoseSender() : as(n, "movRobToScanPos", boost::bind(&RobotPoseSender::executeCB, this, _1), false){
      as.start();   
    }

    void sendRobotComand(double joint_position_command[5]){
      int eki_cmd_buff_len_;
      unsigned char sendbuf[2048];
      int n_dof_ = 6;
      const int EOS_SIZE = 5;
      const int DATA_SIZE = 12;

      TiXmlDocument xml_out;
      TiXmlElement* robot_command = new TiXmlElement("RobotCommand");
      TiXmlElement* pos = new TiXmlElement("Pos");
      TiXmlText* empty_text = new TiXmlText("");
      robot_command->LinkEndChild(pos);
      pos->LinkEndChild(empty_text);   // force <Pos></Pos> format (vs <Pos />)
      char axis_name[] = "A1";
      for (int i = 0; i < n_dof_; ++i)
      {
        //pos->SetAttribute(axis_name, std::to_string(angles::to_degrees(joint_position_command[i])).c_str());
        pos->SetAttribute(axis_name, std::to_string(joint_position_command[i]).c_str());
        axis_name[1]++;
      }
      xml_out.LinkEndChild(robot_command);

      
      TiXmlPrinter xml_printer;
      xml_printer.SetStreamPrinting();  // no linebreaks
      xml_out.Accept(&xml_printer);
      xml_out.SaveFile( "pozicieRobota.xml" );  

      //char str[2];
      //double len = send(sockfd_client,"2",1,0);
      double len = send(sockfd_client ,xml_printer.CStr(), xml_printer.Size(), 0);
      printf("Send vratil = %lf \n",len);

      return;
    }

    void executeCB(const quality_inspection::MovRobToScanPosGoalConstPtr& goal){//, Server* as){
      double joint_position_command[6];  
      joint_position_command[0] = (double) goal->A1;
      joint_position_command[1] = (double) goal->A2;
      joint_position_command[2] = (double) goal->A3;
      joint_position_command[3] = (double) goal->A4;
      joint_position_command[4] = (double) goal->A5;
      joint_position_command[5] = (double) goal->A6;    
      sendRobotComand(joint_position_command);
      //std::cout << goal->A1 << goal->A2 << goal->A3 << goal->A4 << goal->A5 << goal->A6 << std::endl;
      
      quality_inspection::MovRobToScanPosResult robotPos;
      robotPos.x = 1;
      robotPos.y = 2;
      robotPos.z = 3;
      robotPos.A = 4;
      robotPos.B = 5;    
      robotPos.C = 6;
      
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
      server_addr.sin_addr.s_addr = inet_addr("192.168.1.2");
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
  
      printf("Client connected \n");
    }
  };
  
  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "movRobToScanPosServer");

    RobotPoseSender robotPoseSender;
    robotPoseSender.initSocket();

     
    ros::spin();
    return 0;
  }