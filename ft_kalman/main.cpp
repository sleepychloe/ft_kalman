/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   main.cpp                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/24 01:27:37 by yhwang            #+#    #+#             */
/*   Updated: 2024/04/24 02:08:58 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstring>
#include <unistd.h>
#include "./Color.hpp"

int	main(int argc, char **argv)
{
	if (argc != 1)
	{
		std::cerr << RED << "error: invalid argument" << BLACK << std::endl;
		return (1);
	}
	(void)argv;

	int	client_sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (client_sock < 0)
	{
		std::cerr << RED << "error: socket creation failed" << BLACK << std::endl;
		return (1);
	}
	std::cout << CYAN << "socket successfully created" << BLACK << std::endl;
	
	struct sockaddr_in	servaddr;
	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = htonl(2130706433); //127.0.0.1
	servaddr.sin_port = htons(4242);

	const char*	handshake = "READY";
	sendto(client_sock, handshake, strlen(handshake), 0, (struct sockaddr*)&servaddr, sizeof(servaddr));

	char	buf[1024];
	int	i = 0;//
	while (1)
	{
		ssize_t	recieve = recvfrom(client_sock, buf, sizeof(buf), 0, NULL, NULL);
		if (recieve < 0)
		{
			std::cerr << RED << "error: failed to recieve from server" << BLACK << std::endl;
			close(client_sock);
			return (1);
		}

		buf[recieve] = '\0';
		std::cout << i++ << ": " << buf << std::endl;
	}

	close(client_sock);
	return (0);
}
