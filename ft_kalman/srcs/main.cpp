/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   main.cpp                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/24 01:27:37 by yhwang            #+#    #+#             */
/*   Updated: 2024/04/28 19:21:07 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../incs/Parse.hpp"
#include "../incs/Utils.hpp"
#include "../incs/KalmanFilter.hpp"

# define	DT			0.01
# define 	ACCELEROMETER_NOISE	0.001
# define	GYROSCOPE_NOISE		0.01
# define	GPS_NOISE		0.1

bool	g_running_flag;

int	main(int argc, char **argv)
{
	if (argc != 1)
	{
		std::cerr << RED << "error: invalid argument" << BLACK << std::endl;
		return (1);
	}
	(void)argv;

	g_running_flag = true;

	int			client_sock = create_sock();
	struct sockaddr_in	servaddr = create_sockaddr_in(4242);

	signal(SIGINT, signal_handler);
    	signal(SIGQUIT, signal_handler);

	std::string	msg = "READY";
	if (!send_msg(client_sock, servaddr, msg) || !is_serv_available(client_sock, 1))
		return (close(client_sock), 1);
	std::cout << CYAN << "successfully sent message to server: " << msg << BLACK << std::endl;

	std::string	buf;
	Parse		p;
	int		i = -1;

	while (g_running_flag && ++i < 7)
	{		
		if (!recv_from_serv(client_sock, buf) || !is_serv_available(client_sock, 3))
			return (close(client_sock), 1);
		std::cout << buf << std::endl;
		p.parse(buf);
	}
	std::cout << YELLOW << "initial values" << BLACK << std::endl;
	p.print();

	msg = std::to_string(p.getPos()[0]) + " "
		+ std::to_string(p.getPos()[1]) + " "
		+ std::to_string(p.getPos()[2]);
	if (!send_msg(client_sock, servaddr, msg) || !is_serv_available(client_sock, 3))
		return (close(client_sock), 1);
	std::cout << CYAN << "successfully sent message to server: " << std::endl
		<< std::setprecision(std::numeric_limits<long double>::digits10)
		<< p.getPos()[0] << " " << p.getPos()[1] << " " << p.getPos()[2] << BLACK << std::endl;
	
	Vector<double>	init_state({p.getPos()[0], p.getPos()[1], p.getPos()[2], p.getSpeed(), 0, 0});
	Matrix<double>	init_covariance({{pow(GPS_NOISE, 2), 0, 0, 0, 0, 0},
						{0, pow(GPS_NOISE, 2), 0, 0, 0, 0},
						{0, 0, pow(GPS_NOISE, 2), 0, 0, 0},
						{0, 0, 0, pow(ACCELEROMETER_NOISE * GYROSCOPE_NOISE, 2), 0, 0},
						{0, 0, 0, 0, pow(ACCELEROMETER_NOISE * GYROSCOPE_NOISE, 2), 0},
						{0, 0, 0, 0, 0, pow(ACCELEROMETER_NOISE * GYROSCOPE_NOISE, 2)}});
	Matrix<double>	transition_matrix({{1, 0, 0, DT, 0, 0},
						{0, 1, 0, 0, DT, 0},
						{0, 0, 1, 0, 0, DT},
						{0, 0, 0, 1, 0, 0},
						{0, 0, 0, 0, 1, 0},
						{0, 0, 0, 0, 0, 1}});
	Matrix<double>	observation_matrix({{1, 0, 0, 0 ,0 ,0},
						{0, 1, 0, 0, 0, 0},
						{0, 0, 1, 0, 0, 0},
						{0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0},
						{0, 0, 0, 0, 0, 0}});
	Matrix<double>	process_noise_covariance({{pow(GPS_NOISE, 2), 0, 0, 0, 0, 0},
						{0, pow(GPS_NOISE, 2), 0, 0, 0, 0},
						{0, 0, pow(GPS_NOISE, 2), 0, 0, 0},
						{0, 0, 0, pow(ACCELEROMETER_NOISE * GYROSCOPE_NOISE, 2), 0, 0},
						{0, 0, 0, 0, pow(ACCELEROMETER_NOISE * GYROSCOPE_NOISE, 2), 0},
						{0, 0, 0, 0, 0, pow(ACCELEROMETER_NOISE * GYROSCOPE_NOISE, 2)}});
	Matrix<double>	measurement_noise_covariance({{pow(GPS_NOISE, 2), 0, 0, 0, 0, 0},
						{0, pow(GPS_NOISE, 2), 0, 0, 0, 0},
						{0, 0, pow(GPS_NOISE, 2), 0, 0, 0},
						{0, 0, 0, pow(ACCELEROMETER_NOISE * GYROSCOPE_NOISE, 2), 0, 0},
						{0, 0, 0, 0, pow(ACCELEROMETER_NOISE * GYROSCOPE_NOISE, 2), 0},
						{0, 0, 0, 0, 0, pow(ACCELEROMETER_NOISE * GYROSCOPE_NOISE, 2)}});

	KalmanFilter<double>	kalman(init_state, init_covariance,
					transition_matrix, observation_matrix,
					process_noise_covariance, measurement_noise_covariance);
	std::vector<double>	pos = p.getPos();

	while (g_running_flag)
	{
		/* predict */
		for (size_t i = 0; i < 299; i++)
		{
			while (buf.find("ACCELERATION") == std::string::npos)
			{
				if (!recv_from_serv(client_sock, buf) || !is_serv_available(client_sock, 3))
					return (close(client_sock), 1);
				std::cout << buf << std::endl;
			}
			p.parse(buf);
			while (buf.find("DIRECTION") == std::string::npos)
			{
				if (!recv_from_serv(client_sock, buf) || !is_serv_available(client_sock, 3))
					return (close(client_sock), 1);
				std::cout << buf << std::endl;
			}
			p.parse(buf);
			p.computeVelocity();

			kalman.predict();
			std::cout << "predict" << std::endl;
			pos[0] = kalman.getState().getVector()[0];
			pos[1] = kalman.getState().getVector()[1];
			pos[2] = kalman.getState().getVector()[2];
			msg = std::to_string(pos[0]) + " "
				+ std::to_string(pos[1]) + " "
				+ std::to_string(pos[2]);
			if (!send_msg(client_sock, servaddr, msg) || !is_serv_available(client_sock, 3))
				return (close(client_sock), 1);
			std::cout << CYAN << "successfully sent message to server: " << std::endl
				<< std::setprecision(std::numeric_limits<long double>::digits10)
				<< pos[0] << " " << pos[1] << " " << pos[2] << BLACK << std::endl;
		}

		/* update */
		while (buf.find("POSITION") == std::string::npos)
		{
			if (!recv_from_serv(client_sock, buf) || !is_serv_available(client_sock, 3))
				return (close(client_sock), 1);
			std::cout << buf << std::endl;
		}
		p.parse(buf);

		while (buf.find("ACCELERATION") == std::string::npos)
		{
			if (!recv_from_serv(client_sock, buf) || !is_serv_available(client_sock, 3))
				return (close(client_sock), 1);
			std::cout << buf << std::endl;
		}
		p.parse(buf);
		while (buf.find("DIRECTION") == std::string::npos)
		{
			if (!recv_from_serv(client_sock, buf) || !is_serv_available(client_sock, 3))
				return (close(client_sock), 1);
			std::cout << buf << std::endl;
		}
		p.parse(buf);
		p.computeVelocity();
		
		std::cout << RED << "position" << BLACK << std::endl;
		Vector<double>	measurement({p.getPos()[0], p.getPos()[1], p.getPos()[2], p.getVelocity()[0], p.getVelocity()[1], p.getVelocity()[2]});
		kalman.update(measurement);
		std::cout << "update" << std::endl;

		if (!send_msg(client_sock, servaddr, msg) || !is_serv_available(client_sock, 3))
			return (close(client_sock), 1);
		std::cout << CYAN << "successfully sent message to server: " << std::endl
			<< std::setprecision(std::numeric_limits<long double>::digits10)
			<< pos[0] << " " << pos[1] << " " << pos[2] << BLACK << std::endl;
	}
	close(client_sock);
	return (0);
}
