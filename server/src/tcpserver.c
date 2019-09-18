/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.


   Changed by: martijnvwezel@muino.nl to make it multi-threaded

   To do:
    -> Solution for the client[socket ] = accept thing to kill unused sockets
    -> malloc variable BOT_ID[] issues writing two at once

*/

#include <arpa/inet.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <pthread.h> /*For multithreading*/
#include <unistd.h>  /* close */

#include "AdhocServer.h"
#include "CommandList.h"

#define __DEBUG__ 1
#define MAX_LISTEN 4 // total number of cars you are waiting for
#define true 1
#define false 0

unsigned int con_count = 0;
int socket_desc, c, read_size;
struct sockaddr_in server, client;

int cmd_val = 0;
int val;
double prop_fac;
int ret = 0;

pthread_t tid;

pthread_t tid2;

int command_case()
{

  printf("Enter Bot ID to send the packet\n");
  // COMMAND THIS LINE WHEN USING ONLY ONE BOT (and dst_id=15)
  scanf("%d", &dst_id);
  int inlist = false;
  for (int i = 0; i < 4; ++i)
  {
    if (dst_id == BOT_ID[i])
    {
      inlist = true;
    }
  }

  if (!inlist)
  {
    printf("Not given in the list bot_id%d \nChoose between this:\n", dst_id);
    for (int i = 0; i < 4; ++i)
    {

      printf("|Bot with ID [%d] : on Socknumber <%d> Connected\n", BOT_ID[i],
             client_sock[i]);
    }
    return 0;
  }

  // dst_id=15;
  printf("Enter the command(1-12) to the bot-%d : \n", dst_id);
  printf("  1. Move forward \n");
  printf("  2. Move forward for time in seconds \n");
  printf("  3. Move reverse \n");
  printf("  4. Move reverse for time in seconds \n");
  printf("  5. Move left time\n");
  printf("  6. Move right time\n");
  printf("  7. Stop the bot\n");
  //       printf("  8. Get obstacle distance left \n");
  //       printf("  9. Get obstacle distance right\n");
  printf("  8. Get obstacle distance front\n");
  printf("  9. Get RSSI value\n");
  printf("  10. Get ID\n");
  printf("  11. Execute commands from file (cmd_file.txt)\n");
  printf("  12. Execute 30 sec every 1 sec drive and RSSI sample\n");
  printf("  13. propagation factor RSSI samples\n");
  printf("  14. Get rpl data\n");
  printf(" Waiting for user input : ");

  scanf("%d", &cmd_val);
  char charchecker = '\n';
  char charchecker2 = '\r';

  if ((((int)charchecker) == cmd_val) || (((int)charchecker) == cmd_val))
  {
    printf("saw and enter\n");
    cmd_val = 14;
  }

  switch (cmd_val)
  {

  case 1:
    send_forward_time(src_id, dst_id, 0);
    break;
  case 2:
    printf("Enter the time in seconds : \n");
    scanf("%d", &val);
    send_forward_time(src_id, dst_id, val);
    break;
  case 3:
    send_reverse_time(src_id, dst_id, 0);
    break;
  case 4:
    printf("Enter the time in seconds : \n");
    scanf("%d", &val);
    send_reverse_time(src_id, dst_id, val);
    break;
  case 5:
    printf("Enter the time for left turn in seconds : \n");
    scanf("%d", &val);
    send_rotate_left(src_id, dst_id, val);
    break;
  case 6:
    printf("Enter the time for right movement : \n");
    scanf("%d", &val);
    send_rotate_right(src_id, dst_id, val);
    break;
  case 7:
    printf("Sending command to stop the bot\n");
    stop_bot(src_id, dst_id);
    break;
    /*            case 8:
                printf("Fetchng left obstacle sensor information  \n");
                printf("Left Obtacle sensor reading :
       %d\n",get_obstacle_data(src_id,dst_id,ULTRASONIC_LEFT)); break; case 9:
                printf("Fetchng right  obstacle sensor information \n");
                printf("Right Obtacle sensor reading :
       %d\n",get_obstacle_data(src_id,dst_id,ULTRASONIC_RIGHT)); break; */
  case 8:
    printf("Fetchng front obstacle sensor information \n");
    printf("Front Obtacle sensor reading : %d\n",
           get_obstacle_data(src_id, dst_id, ULTRASONIC_FRONT));
    break;
  case 9:
    printf("Fetchng RSSI information \n");
    printf("RSSI reading : %ld\n", get_RSSI(src_id, dst_id));
    break;
  case 10:
    printf("Fetchng ID\n");
    printf("ID of the bot : %d\n", BOT_ID[bot_num]);
    break;
  case 11:
    read_file();
    break;
  case 12:
    printf("RSSI measurement program\n");
    // send_forward_time(src_id, dst_id, 7);
    printf("RSSI reading : {\n");
    printf("%ld\n", get_RSSI(src_id, dst_id));
    send_forward_time(src_id, dst_id, 1);
    int i, j = 0;
    printf("[\n");
    for (i = 0; i < 5; i++)
    {

      send_forward_time(src_id, dst_id, 1);
      printf("[");
      printf("%ld", get_RSSI(src_id, dst_id));
      for (j = 0; j < 9; j++)
      {
        printf(",%ld", get_RSSI(src_id, dst_id));
      }
      printf("],\n");
      sleep(1);
    }
    printf("},\n");
    // printf("RSSI reading : %ld\n",get_RSSI(src_id,dst_id));
    break;
  case 13:
    printf("RSSI continuously measurement program\n");
    printf("Enter the propagation factor:\n");
    scanf("%lf", &prop_fac);
    int k = 0;
    printf("[\n");
    for (k = 0; k < 5; k++)
    {
      printf("[");

      double propagation_factor =
          prop_fac; // NLOS situation, see paper in sources
      printf("%ld,\n", get_RSSI(src_id, dst_id));
      double rssi1 = 1.1; // REMOVED CALCULATION
      double rssi2 =
          pow(10, rssi1);   // Calculate d, added math.h and -lm in compiler
      printf("%lf", rssi2); // Print d
      printf("],\n");
      sleep(1);
    }
    break;

  case 14:
    get_rpldata(dst_id); // in progress routing protocol, not implemented
    break;
  default:
    printf("Unknown command received\n");
    break;
  }
  return 0;
}

void *reader()
{

  while (1)
  {
    int socktoread = 0;
    for (int i = 0; i < 4; ++i)
    {
      if (BOT_ID[i] == 15)
        socktoread = client_sock[i];
    }

    if (socktoread == 0)
    {
      continue;
    }
    int ret = 0;
    char *value = NULL;
    int client_index = get_index(dst_id);

    memset(client_message, '\0', 1024);

    printf("Receving from sockfd - %d\n", socktoread);

    ret = recv(socktoread, client_message, 1024, 0);
    value = get_data(client_message);

    printf("data received from rrsi back: %x %x %x %x\n", value[0], value[1],
           value[2], value[3]);

    int rssietje =
        (value[0] << 24) | (value[1] << 16) | (value[2] << 8) | (value[3]);
    printf("rssietje: %d DBm\n", rssietje);
  }

  // char in_buf[1024];
  // int socktoread = 0;
  // while(1){
  // for (int i = 0; i < 4; ++i){
  //        if(BOT_ID[i]==15)
  //          socktoread = client_sock[i];
  //     }
  //   int errorread =   read(socktoread, in_buf, sizeof(in_buf));
  //   printf("Bufferke: [%x]\n",in_buf);
  //   char *value = get_data(in_buf);

  // printf("data received from rrsi back: %x %x %x
  // %x\n",value[0],value[1],value[2],value[3]);

  // //  printf("Bufferke: [%x %x %x %x %x %x %x %x]\n",in_buf[0] ,in_buf[1]
  // ,in_buf[2] ,in_buf[3], in_buf[4] ,in_buf[5], in_buf[6] ,in_buf[7]  );
  // }
  return NULL;
}

void *accept_clients()
{

  while (1)
  { // con_count != NUM_CONNECTIONS
    // for (int i = 0; i < 4; ++i){
    //     printf("#Bot with ID [%d] : on Socknumber <%d>
    //     Connected\n",BOT_ID[i],client_sock[i]);
    // }

    c = sizeof(struct sockaddr_in);
    // puts("* Waiting for bots to connect");

    // accept connection from an incoming client
    int sockertnum =
        accept(socket_desc, (struct sockaddr *)&client, (socklen_t *)&c);

    if (sockertnum < 0)
    {
      perror("cannot accept connection, because client_socket < 0");
      continue;
      // return 0;
    }
    printf("Got new  sockfd %d\n", sockertnum);
    printf("  - Accepted connection\n");

    client_sock[con_count] = sockertnum;
    int botidtemp = get_botID(con_count);

    printf("botid: %d\n", botidtemp);
    // when no bot is connected he wouldn't go in the for-loop
    if (con_count == 0)
    {
      BOT_ID[con_count] = botidtemp;
      client_sock[con_count] = sockertnum;
    }
    else
    {
      for (int i = 0; i < con_count; ++i)
      { // i++ of ++i makes no difference
        if (BOT_ID[i] != botidtemp)
        {
          if (i == (con_count - 1))
          {
            BOT_ID[con_count] = botidtemp;
            client_sock[con_count] = sockertnum; // todo stop old sock number
          }
        }
        else
        {
          close(client_sock[i]); // todo stop old sock number somehow done now
          client_sock[i] = sockertnum;
          client_sock[con_count] = 0;
          BOT_ID[i] = botidtemp;
          con_count--;
          break;
        }
      }
    }
    // BOT_ID[con_count] = botidtemp;
    // printf("  - Bot with ID : <%d> Connected\n",BOT_ID[con_count]);
    // printf("  - Connected sock : <%d> Connected\n",client_sock[con_count]);
    con_count++;
    printf("-----------------------------------------------------------\n");

    for (int i = 0; i < 4; ++i)
    {
      printf("^Bot with ID [%d] : on Socknumber <%d> Connected\n", BOT_ID[i],
             client_sock[i]);
    }
    printf("\r\n\r\n");

  } // con_count != NUM_CONNECTIONS
  // system("clear");
  return NULL;
  // return 1;//return wehn connection accepted
}

int main(int argc, char *argv[])
{

  int cmd_val = 0;
  int val;
  double prop_fac;
  int ret = 0;
  // int i = 0;

  if (argc != 2)
  {
    printf("Please enter %s  <NUMBER OF BOTS>\n", argv[0]);
    printf("Program start in mode with max 4 bots\n");
    NUM_CONNECTIONS = 4;
    // exit(0);
  }
  else
  {
    NUM_CONNECTIONS = atoi(argv[1]);
  }

  BOT_ID = (char *)malloc(NUM_CONNECTIONS * sizeof(char));
  memset(BOT_ID, 0, NUM_CONNECTIONS);
  client_sock = (int *)malloc(NUM_CONNECTIONS * sizeof(int));
  memset(BOT_ID, 0, NUM_CONNECTIONS * sizeof(int));

  socket_desc = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_desc == -1)
  {
    printf("Could not create socket");
  }
  puts("\n");
  puts("******* Server control program (Adhoc networking course) *******");
  puts("\n");

  puts("* Socket created");

  int option = 1;
  setsockopt(socket_desc, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));
  server.sin_family = AF_INET; // Prepare the sockaddr_in structure
  server.sin_addr.s_addr = INADDR_ANY;
  server.sin_port = htons(TCP_PORT); // TCP_PORT ADHOC_UDP_PORT

  // Bind
  if (bind(socket_desc, (struct sockaddr *)&server, sizeof(server)) < 0)
  {
    perror("Bind failed. Error");
    return 1;
  }
  puts("* Binding done");

  // Listen
  listen(socket_desc, MAX_LISTEN);

  int err = pthread_create(&(tid), NULL, &accept_clients, NULL);
  int err2 = 0; // pthread_create(&(tid2), NULL, &reader, NULL);
  if (err != 0 || err2 != 0)
  {
    printf("\ncan't create thread :[%s]", strerror(err));
    printf("\ncan't create thread :[%s]",
           strerror(err2)); // HIERM OET DIE ACCEPTION STAAN
    exit(0);
  }
  else
  {
    while (1)
    {
      int dd = command_case();
      // printf("error %d\n",dd );
    }
  } // end else

  close(*client_sock); // close clientsocket
  close(socket_desc);  // if not closing socket stays open

  return 0;
}
