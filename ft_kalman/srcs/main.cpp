/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   main.cpp                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/24 01:27:37 by yhwang            #+#    #+#             */
/*   Updated: 2024/04/26 17:13:56 by yhwang           ###   ########.fr       */
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

	std::vector<double>	tmp = p.getDir();
	tmp.push_back(p.getSpeed());
	Vector<double>	init_state(tmp);
	Matrix<double>	init_covariance({{1, 0, 0, 0},
					{0, 1, 0, 0},
					{0, 0, 1, 0},
					{0, 0, 0, 1}});
	Matrix<double>	transition_matrix({{1, 0, 0, DT},
					{0, 1, 0, 0},
					{0, 0, 1, 0},
					{0, 0, 0, 1}});
	Matrix<double>	observation_matrix({{1, 0, 0, 0},
					{0, 1, 0, 0},
					{0, 0, 1, 0},
					{0, 0, 0, 1}});
	Matrix<double>	process_accelerometer_noise({{(pow(DT, 4) / 4) * pow(ACCELEROMETER_NOISE, 2), (pow(DT, 3) / 2) * pow(ACCELEROMETER_NOISE, 2), 0, (pow(DT, 3) / 2) * pow(ACCELEROMETER_NOISE, 2)},
						{(pow(DT, 3) / 2) * pow(ACCELEROMETER_NOISE, 2), pow(DT, 2) * pow(ACCELEROMETER_NOISE, 2), 0, pow(DT, 2) * pow(ACCELEROMETER_NOISE, 2)},
						{0, 0, (pow(DT, 4) / 4) * pow(ACCELEROMETER_NOISE, 2), 0},
						{(pow(DT, 3) / 2) * pow(ACCELEROMETER_NOISE, 2), pow(DT, 2) * pow(ACCELEROMETER_NOISE, 2), 0, pow(DT, 2) * pow(ACCELEROMETER_NOISE, 2)}});
	Matrix<double>	process_gyroscope_noise({{(pow(DT, 4) / 4) * pow(GYROSCOPE_NOISE, 2), (pow(DT, 3) / 2) * pow(GYROSCOPE_NOISE, 2), 0, (pow(DT, 3) / 2) * pow(GYROSCOPE_NOISE, 2)},
						{(pow(DT, 3) / 2) * pow(GYROSCOPE_NOISE, 2), pow(DT, 2) * pow(GYROSCOPE_NOISE, 2), 0, pow(DT, 2) * pow(GYROSCOPE_NOISE, 2)},
						{0, 0, (pow(DT, 4) / 4) * pow(GYROSCOPE_NOISE, 2), 0},
						{(pow(DT, 3) / 2) * pow(GYROSCOPE_NOISE, 2), pow(DT, 2) * pow(GYROSCOPE_NOISE, 2), 0, pow(DT, 2) * pow(GYROSCOPE_NOISE, 2)}});
	Matrix<double>	process_noise_covariance = process_accelerometer_noise.mul_mat(process_gyroscope_noise);
	Matrix<double>	measurement_noise_covariance({{pow(GPS_NOISE, 2), 0, 0},
						{0, pow(GPS_NOISE, 2), 0},
						{0, 0, pow(GPS_NOISE, 2)}});

	KalmanFilter<double>	kalman(init_state, init_covariance, transition_matrix, observation_matrix, process_noise_covariance, measurement_noise_covariance);
	kalman.predict();

	msg = std::to_string(kalman.getState().getVector()[0]) + " "
		+ std::to_string(kalman.getState().getVector()[1]) + " "
		+ std::to_string(kalman.getState().getVector()[2]);
	if (!send_msg(client_sock, servaddr, msg) || !is_serv_available(client_sock, 3))
		return (close(client_sock), 1);

	while (g_running_flag)
	{
		if (!recv_from_serv(client_sock, buf) || !is_serv_available(client_sock, 3))
			return (close(client_sock), 1);
		std::cout << buf << std::endl;
		p.parse(buf);
		Vector<double>	measurement({p.getPos()[0], p.getPos()[1], p.getPos()[2]});

		kalman.predict();
		kalman.update(measurement);
		msg = std::to_string(kalman.getState().getVector()[0]) + " "
			+ std::to_string(kalman.getState().getVector()[1]) + " "
			+ std::to_string(kalman.getState().getVector()[2]);
		if (!send_msg(client_sock, servaddr, msg) || !is_serv_available(client_sock, 3))
			return (close(client_sock), 1);
	}
	close(client_sock);
	return (0);
}
