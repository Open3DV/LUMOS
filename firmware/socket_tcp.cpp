#include "socket_tcp.h"
#include "protocol.h"
#include "easylogging++.h"


int setup_socket(int port)
{
    int server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(server_sock<0)
    {
        perror("ERROR: socket()");
        exit(0);
    }

    int flags = 3;
    setsockopt(server_sock, SOL_TCP, TCP_KEEPIDLE, (void*)&flags, sizeof(flags));
    flags = 3;
    setsockopt(server_sock, SOL_TCP, TCP_KEEPCNT, (void*)&flags, sizeof(flags));
    flags = 1;
    setsockopt(server_sock, SOL_TCP, TCP_KEEPINTVL, (void*)&flags, sizeof(flags));


    //将套接字和IP、端口绑定
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));  //每个字节都用0填充
    serv_addr.sin_family = AF_INET;  //使用IPv4地址
    serv_addr.sin_addr.s_addr = INADDR_ANY;  //具体的IP地址
    serv_addr.sin_port = htons(port);  //端口
    int ret = bind(server_sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
    if(ret==-1)
    {
        printf("bind ret=%d, %s\n", ret, strerror(errno));
        close(server_sock);
        return DF_FAILED;
    }

    //进入监听状态，等待用户发起请求
    ret = listen(server_sock, 1);
    if(ret == -1)
    {
        printf("listen ret=%d, %s\n", ret, strerror(errno));
        close(server_sock);
        return DF_FAILED;
    }
    return server_sock;
}


int accept_new_connection(int server_sock)
{
    //std::cout<<"listening"<<std::endl;
    //接收客户端请求
    struct sockaddr_in clnt_addr;
    socklen_t clnt_addr_size = sizeof(clnt_addr);
    int client_sock = accept(server_sock, (struct sockaddr*)&clnt_addr, &clnt_addr_size);

    //print address
    char buffer[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &clnt_addr.sin_addr, buffer, sizeof(buffer));

    struct timeval timeout = {1,0};
    int ret = setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
    ret = setsockopt(client_sock, SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(timeout));
    //int flags = 1;
    //setsockopt(client_sock, SOL_SOCKET, SO_KEEPALIVE, (void*)&flags, sizeof(flags));

    //std::cout<<"accepted connection from "<<buffer<<std::endl;
    
    return client_sock;
}


int send_buffer(int sock, const char* buffer, int buffer_size)
{
   /* 
  struct tcp_info info; 
  int len=sizeof(info); 
  getsockopt(sock, IPPROTO_TCP, TCP_INFO, &info, (socklen_t *)&len); 
  if((info.tcpi_state==TCP_ESTABLISHED))
  {
         LOG(INFO)<<"ok";
  }
  else
  {
	 return DF_FAILED; 
  }

  */

 
	
	
    int size = 0;
    int ret = send(sock, (char *)&buffer_size, sizeof(buffer_size), MSG_NOSIGNAL);
    LOG(INFO) << "send buffer_size ret=" << ret;
    if (ret == -1)
    {
        return DF_FAILED;
    }

    int sent_size = 0;
    ret = send(sock, buffer, buffer_size, MSG_NOSIGNAL);
    LOG(INFO) << "send buffer ret=" << ret;
    if (ret == -1)
    {
        return DF_FAILED;
    }
    sent_size += ret;
    while (sent_size != buffer_size)
    {
        buffer += ret;
        LOG(INFO) << "sent_size=" << sent_size;
        ret = send(sock, buffer, buffer_size - sent_size, MSG_NOSIGNAL);
        LOG(INFO) << "ret=" << ret;
        if (ret == -1)
        {
            return DF_FAILED;
        }
        sent_size += ret;
    }

    return DF_SUCCESS;
}

int recv_buffer(int sock, char* buffer, int buffer_size)
{
    int size = 0;
    int ret = recv(sock, (char*)&size, sizeof(size), 0);
  
    if(buffer_size < size)
    {
        LOG(ERROR)<<"buffer_size < size";
        LOG(INFO)<<"buffer_size= "<<buffer_size;
        LOG(INFO)<<"size= "<<size;
        return DF_FAILED; 
    }
    //assert(buffer_size >= size);
    int n_recv = 0;
    // ret = DF_SUCCESS;

    while (ret > 0)
    {
        ret = recv(sock, buffer, buffer_size, 0);
        //std::cout << "ret="<<ret << std::endl;
        if (ret > 0)
        {
            buffer_size -= ret;
            n_recv += ret;
            buffer += ret;
        }

        if (buffer_size == 0)
        {
            //assert(n_recv == size);
            if (n_recv != size)
			{
				LOG(ERROR) << "recv err: n_recv != size ";
				return DF_FAILED; 
			}
            return DF_SUCCESS;
        }
    }

    LOG(INFO) << "recv err,ret= " << ret;
    return DF_FAILED;
}

int send_command(int sock, int command)
{
    return send_buffer(sock, (const char*)&command, sizeof(int));
}

int recv_command(int sock, int* command)
{
    return recv_buffer(sock, (char*)command, sizeof(int));
}