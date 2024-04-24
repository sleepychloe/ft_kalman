/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   Utils.cpp                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/24 12:37:34 by yhwang            #+#    #+#             */
/*   Updated: 2024/04/24 14:37:28 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "./Utils.hpp"

void	signal_handler(int signo)
{
	std::cerr << RED;
	if (signo == SIGINT)
		std::cerr << "SIGINT(Ctrl-C)";
	else if (signo == SIGQUIT)
		std::cerr << "SIGQUIT(Ctrl-\\)";
	std::cerr << " detected" << BLACK << std::endl;
	g_running_flag = false;
}

int	create_sock(void)
{
	int	sock = socket(AF_INET, SOCK_DGRAM, 0);

	if (sock < 0)
	{
		std::cerr << RED << "error: socket creation failed" << BLACK << std::endl;
		exit(1);
	}
	std::cout << CYAN << "socket successfully created" << BLACK << std::endl;
	return (sock);
}

struct sockaddr_in	create_sockaddr_in(int port)
{
	struct sockaddr_in	addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(2130706433); //127.0.0.1
	addr.sin_port = htons(port);
	return (addr);
}

bool	send_handshake(int sock, struct sockaddr_in servaddr)
{
	const char*	handshake = "READY";
	ssize_t		byte = sendto(sock, handshake, strlen(handshake), 0, (struct sockaddr*)&servaddr, sizeof(servaddr));
	if (byte < 0)
	{
		std::cerr << RED << "error: cannot send handshake" << BLACK << std::endl;
		return (false);
	}
	std::cout << CYAN << "successfully sent handshake to server" << BLACK << std::endl;
	return (true);
}

bool	is_serv_available(int sock, int time_sec)
{
	fd_set	readfd;
	FD_ZERO(&readfd);
	FD_SET(sock, &readfd);

	struct timeval	timeout;
	timeout.tv_sec = time_sec;
	timeout.tv_usec = 0;
	int	ready = select(sock + 1, &readfd, NULL, NULL, &timeout);
	if (ready < 0)
	{
		std::cerr << RED << "error: select() failed" << BLACK << std::endl;
		return (false);
	}
	else if (ready == 0)
	{
		std::cerr << RED << "error: server is not responding(timeout)" << BLACK << std::endl;
		return (false);
	}
	return (true);
}

bool	recv_from_serv(int sock, std::string &buf)
{
	char	tmp[1024];
	ssize_t	byte = recvfrom(sock, tmp, sizeof(tmp), 0, NULL, NULL);
	if (byte < 0)
	{
		std::cerr << RED << "error: failed to recieve from server" << BLACK << std::endl;
		return (false);
	}
	tmp[byte] = '\0';
	buf = tmp;
	std::cout << buf << std::endl;
	return (true);
}
