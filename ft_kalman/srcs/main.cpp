/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   main.cpp                                           :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/04/24 01:27:37 by yhwang            #+#    #+#             */
/*   Updated: 2024/05/20 09:43:07 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../incs/Parse.hpp"
#include "../incs/KalmanFilter.hpp"
#include "../incs/Utils.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "../incs/OpenGLUtils.hpp"

bool	g_running_flag;

GLFWwindow* window;
GLuint VAO, VBO;
std::vector<Vertex> vertices;

void initOpenGL() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        exit(EXIT_FAILURE);
    }

    window = glfwCreateWindow(800, 600, "3D Graph", NULL, NULL);
    if (!window) {
        glfwTerminate();
        std::cerr << "Failed to create GLFW window\n";
        exit(EXIT_FAILURE);
    }
    glfwMakeContextCurrent(window);
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW\n";
        exit(EXIT_FAILURE);
    }

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * vertices.size(), nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void setupView() {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (double)800 / (double)600, 0.1, 5000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(1000.0, 1000.0, 1000.0,  // Camera position in World Space
              0.0, 0.0, 0.0,  // and looks at the origin
              0.0, 0.0, -1.0); // Head is up (set to 0,-1,0 to look upside-down)
}

void updateGraph(const std::vector<double>& position) {
    Vertex v = {static_cast<GLfloat>(position[0]), static_cast<GLfloat>(position[1]), static_cast<GLfloat>(position[2])};
    vertices.push_back(v);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);
}

void drawAxes(float length) {
    glBegin(GL_LINES);

    // X axis in red
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(length, 0.0, 0.0);

    // Y axis in green
    glColor3f(0.0, 1.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, length, 0.0);

    // Z axis in blue
    glColor3f(0.0, 0.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, length);
    glEnd();
}

void render() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    setupView();  // Set up the camera and projection settings
    
    drawAxes(4000.0);  // Draw axes with a specified length

    // Bind and draw your vertices as before
    glBindVertexArray(VAO);
    glPointSize(2.0f);  // Increase point size for visibility
    glEnableVertexAttribArray(0);
    glDrawArrays(GL_POINTS, 0, vertices.size());
    glBindVertexArray(0);

    glfwSwapBuffers(window);
}

int	main(int argc, char **argv)
{
	if (argc != 1)
	{
		std::cerr << RED << "error: invalid argument" << BLACK << std::endl;
		return (1);
	}
	(void)argv;

	g_running_flag = true;

	int			client_sock = createSock();
	struct sockaddr_in	servaddr = createServAddr(4242);

	signal(SIGINT, signalHandler);
    	signal(SIGQUIT, signalHandler);

	std::string	msg = "READY";
	if (!sendMsg(client_sock, servaddr, msg) || !isServAvailable(client_sock, 1))
		return (close(client_sock), 1);
	std::cout << CYAN << "successfully sent message to server: " << msg << BLACK << std::endl;

	std::string	buf;
	Parse		p;

	for (size_t i = 0; g_running_flag && i < 7; i++)
	{		
		if (!recvFromServ(client_sock, buf) || !isServAvailable(client_sock, 1))
			return (close(client_sock), 1);
		std::cout << buf << std::endl;
		p.parse(buf);
	}

	std::vector<double>	velocity({p.getSpeed(), 0, 0});
	computeVelocity(p.getDir(), p.getAcc(), velocity);

	KalmanFilter<double>	kalman;
	initFilter(p, velocity, kalman);

	if (!sendPos(client_sock, servaddr, kalman.getState().getVector(), 1))
		return (close(client_sock), 1);

    initOpenGL();
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    setupView();
    std::vector<double> pos({kalman.getState().getVector()[0], kalman.getState().getVector()[1], kalman.getState().getVector()[2]});
    updateGraph(pos);
    render();

	Vector<double>	control_input;
	Vector<double>	measurement;

	while (!glfwWindowShouldClose(window) && g_running_flag)
	{
        glfwPollEvents();
		/* predict */
		for (size_t i = 0; i < 299; i++)
		{
			if (!parseElement(client_sock, p, "ACCELERATION")
				|| !parseElement(client_sock, p, "DIRECTION"))
				return (close(client_sock), 1);
			computeVelocity(p.getDir(), p.getAcc(), velocity);

			/* control input: n */
			control_input = Vector<double>({p.getAcc()[0], p.getAcc()[1], p.getAcc()[2]});
			kalman.predict(control_input);
			if (!sendPos(client_sock, servaddr, kalman.getState().getVector(), 1))
				return (close(client_sock), 1);
            pos = std::vector<double>({kalman.getState().getVector()[0], kalman.getState().getVector()[1], kalman.getState().getVector()[2]});
            updateGraph(pos);
            render();
		}
		/* update */
		if (!parseElement(client_sock, p, "POSITION")
			|| !parseElement(client_sock, p, "ACCELERATION")
			|| !parseElement(client_sock, p, "DIRECTION"))
			return (close(client_sock), 1);
		computeVelocity(p.getDir(), p.getAcc(), velocity);

		control_input = Vector<double>({p.getAcc()[0], p.getAcc()[1], p.getAcc()[2]});
		kalman.predict(control_input);

		/* measurement: m */
		measurement = Vector<double>({p.getPos()[0], p.getPos()[1], p.getPos()[2]});
		kalman.update(measurement);
		if (!sendPos(client_sock, servaddr, kalman.getState().getVector(), 1))
			return (close(client_sock), 1);
	}
	close(client_sock);
    glfwDestroyWindow(window);
    glfwTerminate();
	return (0);
}
