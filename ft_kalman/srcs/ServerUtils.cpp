/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ServerUtils.cpp                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/24 12:37:34 by yhwang            #+#    #+#             */
/*   Updated: 2024/06/12 21:13:25 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../incs/Utils.hpp"

void	signalHandler(int signo)
{
	std::cerr << RED;
	if (signo == SIGINT)
		std::cerr << "SIGINT(Ctrl-C)";
	else if (signo == SIGQUIT)
		std::cerr << "SIGQUIT(Ctrl-\\)";
	std::cerr << " detected" << BLACK << std::endl;
	g_running_flag = false;
}

int	createSock(void)
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

struct sockaddr_in	createServAddr(int port)
{
	struct sockaddr_in	addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(2130706433); //127.0.0.1
	addr.sin_port = htons(port);
	return (addr);
}

bool	sendMsg(int sock, struct sockaddr_in servaddr, std::string msg)
{
	ssize_t		byte = sendto(sock, msg.c_str(), msg.length(), 0, (struct sockaddr*)&servaddr, sizeof(servaddr));

	if (byte < 0)
	{
		std::cerr << RED << "error: cannot send message to server" << BLACK << std::endl;
		return (false);
	}
	return (true);
}

bool	isServAvailable(int sock, int time_sec)
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

bool	recvFromServ(int sock, std::string &buf)
{
	char	tmp[1024 * 255];
	ssize_t	byte = recvfrom(sock, tmp, sizeof(tmp), 0, NULL, NULL);

	if (byte < 0)
	{
		std::cerr << RED << "error: failed to recieve from server" << BLACK << std::endl;
		return (false);
	}
	tmp[byte] = '\0';
	buf = tmp;
	return (true);
}

bool	sendPos(int sock, struct sockaddr_in servaddr, std::vector<double> pos, int timeout)
{
	std::string	msg;

	msg = std::to_string(pos[0]) + " " + std::to_string(pos[1]) + " " + std::to_string(pos[2]);
	if (!sendMsg(sock, servaddr, msg) || !isServAvailable(sock, timeout))
		return (false);
	
	std::cout << CYAN << "successfully sent message to server: " << std::endl
		<< std::setprecision(std::numeric_limits<long double>::digits10)
		<< pos[0] << " " << pos[1] << " " << pos[2] << BLACK << std::endl;
	return (true);
}

bool	parseElement(int sock, Parse &p, std::string element, int &end_flag)
{
	std::string	buf;

	while (1)
	{
		if (!recvFromServ(sock, buf) || !isServAvailable(sock, 1))
			return (false);
		std::cout << buf << std::endl;
		if (buf.find(element) != std::string::npos)
		{
			if (element == "DIRECTION"
				&& buf.find(p.getEndtime()) != std::string::npos)
				end_flag++;
			p.parse(buf);
			break ;
		}
	}
	return (true);
}
