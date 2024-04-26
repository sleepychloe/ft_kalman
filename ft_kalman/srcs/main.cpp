/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   main.cpp                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/24 01:27:37 by yhwang            #+#    #+#             */
/*   Updated: 2024/04/27 00:39:07 by yhwang           ###   ########.fr       */
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
	
	Vector<double>	init_state({p.getSpeed() * DT, 0, 0});
	Matrix<double>	init_covariance({{1, 0, 0},
					{0, 1, 0},
					{0, 0, 1}});
	Matrix<double>	transition_matrix({{1, 0, DT},
					{0, 1, 0},
					{0, 0, 1}});
	Matrix<double>	observation_matrix({{1, 0, 0},
					{0, 1, 0},
					{0, 0, 1}});
	Matrix<double>	process_accelerometer_noise({{pow(ACCELEROMETER_NOISE, 2), 0, 0},
						{0, pow(ACCELEROMETER_NOISE, 2), 0},
						{0, 0, pow(ACCELEROMETER_NOISE, 2)}});
	Matrix<double>	process_gyroscope_noise({{pow(GYROSCOPE_NOISE, 2), 0, 0},
						{0, pow(GYROSCOPE_NOISE, 2), 0},
						{0, 0, pow(GYROSCOPE_NOISE, 2)}});

	Matrix<double>	process_noise_covariance = (process_accelerometer_noise + process_gyroscope_noise) * 0.5;
	Matrix<double>	measurement_noise_covariance({{pow(GPS_NOISE, 2), 0, 0},
						{0, pow(GPS_NOISE, 2), 0},
						{0, 0, pow(GPS_NOISE, 2)}});

	KalmanFilter<double>	kalman(init_state, init_covariance, transition_matrix, observation_matrix, process_noise_covariance, measurement_noise_covariance);
	std::vector<double>	pos = p.getPos();
	// pos[0] += init_state.getVector()[0] * DT;
	// pos[1] += init_state.getVector()[1] * DT;
	// pos[2] += init_state.getVector()[2] * DT;
	
	while (g_running_flag)
	{
		if (!recv_from_serv(client_sock, buf) || !is_serv_available(client_sock, 3))
			return (close(client_sock), 1);
		std::cout << buf << std::endl;
		p.parse(buf);

		kalman.predict();
		std::cout << "predict" << std::endl;
		Vector<double>	measurement(p.getAcc());
		kalman.update(measurement * DT);
		std::cout << "update" << std::endl;

		pos[0] += kalman.getState().getVector()[0] * DT;
		pos[1] += kalman.getState().getVector()[1] * DT;
		pos[2] += kalman.getState().getVector()[2] * DT;
		msg = std::to_string(pos[0]) + " "
			+ std::to_string(pos[1]) + " "
			+ std::to_string(pos[2]);
		if (!send_msg(client_sock, servaddr, msg) || !is_serv_available(client_sock, 3))
			return (close(client_sock), 1);
		std::cout << CYAN << "successfully sent message to server: " << std::endl
			<< std::setprecision(std::numeric_limits<long double>::digits10)
			<< pos[0] << " " << pos[1] << " " << pos[2] << BLACK << std::endl;
	}
	close(client_sock);
	return (0);
}
