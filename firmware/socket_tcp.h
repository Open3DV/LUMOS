#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <errno.h> 



int setup_socket(int port);

int accept_new_connection(int server_sock); 

int send_buffer(int sock, const char* buffer, int buffer_size);

int recv_buffer(int sock, char* buffer, int buffer_size);

int send_command(int sock, int command);
 
int recv_command(int sock, int* command);