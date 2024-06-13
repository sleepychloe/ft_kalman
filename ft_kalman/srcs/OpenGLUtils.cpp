/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   OpenGLUtils.cpp                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/05/20 16:06:59 by yhwang            #+#    #+#             */
/*   Updated: 2024/06/14 01:14:56 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../incs/OpenGLUtils.hpp"
#include "../incs/Color.hpp"

bool	init_opengl(t_opengl &ctx)
{
	std::function<void(t_data &)>	init_data = [&](t_data &data)
	{
		data.zoom = 1.0f;
		data.camera_lookat_x = 14500;
		data.camera_lookat_y = -1000;
		data.camera_lookat_z = -2000;
	};

	init_data(ctx.data_position);
	init_data(ctx.data_covariance_p);
	init_data(ctx.data_covariance_v);
	
	if (!glfwInit())
	{
		glfwTerminate();
		glfwDestroyWindow(ctx.window);
		std::cerr << RED << "error: failed to initialize GLFW" << BLACK << std::endl;
		return (false);
	}

	ctx.window = glfwCreateWindow(WINODW_WIDTH, WINDOW_HEIGHT, "Kalman Filter", NULL, NULL);
	if (!ctx.window)
	{
		glfwTerminate();
		glfwDestroyWindow(ctx.window);
		std::cerr << RED << "error: failed to create GLFW window" << BLACK << std::endl;
		return (false);
	}

	glfwMakeContextCurrent(ctx.window);
	glfwSetWindowUserPointer(ctx.window, &ctx);
	glfwSetScrollCallback(ctx.window, scroll_callback);
	glfwSetKeyCallback(ctx.window, key_callback);

	if (glewInit() != GLEW_OK)
	{
		glfwTerminate();
		glfwDestroyWindow(ctx.window);
		std::cerr << RED << "error: failed to initialize GLFW" << BLACK << std::endl;
		return (false);
	}

	std::function<void(GLuint &, GLuint &, GLsizei)>	setup = [&](GLuint &VAO, GLuint &VBO, GLsizei array_size)
	{
		glGenVertexArrays(1, &VAO);
		glBindVertexArray(VAO);
		glGenBuffers(1, &VBO);
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(t_point3) * array_size, nullptr, GL_DYNAMIC_DRAW);

		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(t_point3), (void *)0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	};

	setup(ctx.VAO_position, ctx.VBO_position, ctx.position.size());
	setup(ctx.VAO_covariance_p, ctx.VBO_covariance_p, ctx.covariance_p.size());
	setup(ctx.VAO_covariance_v, ctx.VBO_covariance_v, ctx.covariance_v.size());

	glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // white background
	return (true);
}

void	update_position_graph(t_opengl &ctx, const std::vector<double> &pos)
{
	t_point3	p = {static_cast<GLfloat>(pos[0]), static_cast<GLfloat>(pos[1]), static_cast<GLfloat>(pos[2])};

	ctx.position.push_back(p);
	glBindBuffer(GL_ARRAY_BUFFER, ctx.VBO_position);
	glBufferData(GL_ARRAY_BUFFER, sizeof(t_point3) * ctx.position.size(), ctx.position.data(), GL_DYNAMIC_DRAW);
}

void	update_covariance_graph(t_opengl &ctx, const std::vector<std::vector<double>> &cov)
{
	t_point3	p = {static_cast<GLfloat>(cov[0][0] * 1000000), static_cast<GLfloat>(cov[1][1] * 1000000), static_cast<GLfloat>(cov[2][2] * 1000000)};
	t_point3	v = {static_cast<GLfloat>(cov[3][3] * 100000000), static_cast<GLfloat>(cov[4][4] * 100000000), static_cast<GLfloat>(cov[5][5] * 100000000)};

	ctx.covariance_p.push_back(p);
	glBindBuffer(GL_ARRAY_BUFFER, ctx.VBO_covariance_p);
	glBufferData(GL_ARRAY_BUFFER, sizeof(t_point3) * ctx.covariance_p.size(), ctx.covariance_p.data(), GL_DYNAMIC_DRAW);

	ctx.covariance_v.push_back(v);
	glBindBuffer(GL_ARRAY_BUFFER, ctx.VBO_covariance_v);
	glBufferData(GL_ARRAY_BUFFER, sizeof(t_point3) * ctx.covariance_v.size(), ctx.covariance_v.data(), GL_DYNAMIC_DRAW);
}

void	scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
{
	t_opengl	*ctx = reinterpret_cast<t_opengl *>(glfwGetWindowUserPointer(window));

	int	windowWidth;
	int	windowHeight;
	glfwGetWindowSize(window, &windowWidth, &windowHeight);

	double	cursorX;
	double	cursorY;
	glfwGetCursorPos(window, &cursorX, &cursorY);

	if (cursorX < windowWidth / 2)
	{
		if (yoffset > 0)
			ctx->data_position.zoom *= exp(-0.1);
		else if (yoffset < 0)
			ctx->data_position.zoom *= exp(0.1);
	}
	else
	{
		if (cursorY < windowHeight / 2)
		{
			if (yoffset > 0)
				ctx->data_covariance_p.zoom *= exp(-0.1);
			else if (yoffset < 0)
				ctx->data_covariance_p.zoom *= exp(0.1);
		}
		else
		{
			if (yoffset > 0)
				ctx->data_covariance_v.zoom *= exp(-0.1);
			else if (yoffset < 0)
				ctx->data_covariance_v.zoom *= exp(0.1);
		}
	}
	(void)xoffset;
}

void	key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
	t_opengl	*ctx = reinterpret_cast<t_opengl *>(glfwGetWindowUserPointer(window));

	int	windowWidth;
	int	windowHeight;
	glfwGetWindowSize(window, &windowWidth, &windowHeight);

	double	cursorX;
	double	cursorY;
	glfwGetCursorPos(window, &cursorX, &cursorY);

	if (action == GLFW_PRESS || action == GLFW_REPEAT)
	{
		if (cursorX < windowWidth / 2)
		{
			if (key == GLFW_KEY_A)
				ctx->data_position.camera_lookat_x -= 200;
			else if (key == GLFW_KEY_D)
				ctx->data_position.camera_lookat_x += 200;
			else if (key == GLFW_KEY_S)
				ctx->data_position.camera_lookat_y -= 200;
			else if (key == GLFW_KEY_W)
				ctx->data_position.camera_lookat_y += 200;
			else if (key == GLFW_KEY_DOWN)
				ctx->data_position.camera_lookat_z -= 200;
			else if (key == GLFW_KEY_UP)
				ctx->data_position.camera_lookat_z += 200;
		}
		else
		{
			if (cursorY < windowHeight / 2)
			{
				if (key == GLFW_KEY_A)
					ctx->data_covariance_p.camera_lookat_x -= 200;
				else if (key == GLFW_KEY_D)
					ctx->data_covariance_p.camera_lookat_x += 200;
				else if (key == GLFW_KEY_S)
					ctx->data_covariance_p.camera_lookat_y -= 200;
				else if (key == GLFW_KEY_W)
					ctx->data_covariance_p.camera_lookat_y += 200;
				else if (key == GLFW_KEY_DOWN)
					ctx->data_covariance_p.camera_lookat_z -= 200;
				else if (key == GLFW_KEY_UP)
					ctx->data_covariance_p.camera_lookat_z += 200;
			}
			else
			{
				if (key == GLFW_KEY_A)
					ctx->data_covariance_v.camera_lookat_x -= 200;
				else if (key == GLFW_KEY_D)
					ctx->data_covariance_v.camera_lookat_x += 200;
				else if (key == GLFW_KEY_S)
					ctx->data_covariance_v.camera_lookat_y -= 200;
				else if (key == GLFW_KEY_W)
					ctx->data_covariance_v.camera_lookat_y += 200;
				else if (key == GLFW_KEY_DOWN)
					ctx->data_covariance_v.camera_lookat_z -= 200;
				else if (key == GLFW_KEY_UP)
					ctx->data_covariance_v.camera_lookat_z += 200;
			}
		}
	}
	(void)scancode;
	(void)mods;
}

void	setup_view(t_opengl &ctx, int flag, int view_type)
{
	float	zoom;
	float	camera_lookat_x;
	float	camera_lookat_y;
	float	camera_lookat_z;

	if (flag == POS)
	{
		zoom = ctx.data_position.zoom;
		camera_lookat_x = ctx.data_position.camera_lookat_x;
		camera_lookat_y = ctx.data_position.camera_lookat_y;
		camera_lookat_z = ctx.data_position.camera_lookat_z;
	}
	else if (flag == COV_P)
	{
		zoom = ctx.data_covariance_p.zoom;
		camera_lookat_x = ctx.data_covariance_p.camera_lookat_x;
		camera_lookat_y = ctx.data_covariance_p.camera_lookat_y;
		camera_lookat_z = ctx.data_covariance_p.camera_lookat_z;
	}
	else
	{
		zoom = ctx.data_covariance_v.zoom;
		camera_lookat_x = ctx.data_covariance_v.camera_lookat_x;
		camera_lookat_y = ctx.data_covariance_v.camera_lookat_y;
		camera_lookat_z = ctx.data_covariance_v.camera_lookat_z;
	}

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(50 * zoom, 1 / 1, 1, 10000000000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	if (view_type == 0) // 3d view
		gluLookAt(45000 / zoom, -45000 / zoom, 45000 / zoom, // camera position
			camera_lookat_x / zoom, camera_lookat_y / zoom, camera_lookat_z / zoom, // looks at
			0, 0, 1); // Carmera vector
	if (view_type == 1)
		gluLookAt(0, 0, 90000 / zoom,
			0, 0, 0,
			0, 1, 0);
	if (view_type == 2)
		gluLookAt(0, -90000 / zoom, 0,
			0, 0, 0,
			0, 0, 1);
	(void)ctx;
}

void	draw_axis(double length)
{
	glBegin(GL_LINES);

	glColor3f(1, 0, 0); // red
	glVertex3f(0, 0, 0);
	glVertex3f(length, 0, 0); // +x

	glColor3f(0, 1, 0); // green
	glVertex3f(0, 0, 0);
	glVertex3f(0, length, 0); // +y
	glVertex3f(0, 0, 0);
	glVertex3f(0, -length, 0); // -y

	glColor3f(0, 0, 1); // blue
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, length); // +z
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, -length); // -z

	glEnd();
}

void	draw_graph(t_opengl &ctx, int flag)
{
	GLuint	VAO;
	GLsizei	array_size;
	std::vector<GLfloat> color;
	std::vector<std::vector<int>>	viewport;

	if (flag == POS)
	{
		VAO = ctx.VAO_position;
		array_size = ctx.position.size();
		color = {1, 0, 1};
		viewport = {{0, 200, 400, 400},
				{0, 0, 200, 200},
				{200, 0, 200, 200}};
	}
	else if (flag == COV_P)
	{
		VAO = ctx.VAO_covariance_p;
		array_size = ctx.covariance_p.size();
		color = {0, 0, 0};
		viewport = {{400, 300, 300, 300},
				{700, 450, 150, 150},
				{700, 300, 150, 150}};
	}
	else
	{
		VAO = ctx.VAO_covariance_v;
		array_size = ctx.covariance_v.size();
		color = {0.5, 0.5, 0.5};
		viewport = {{400, 0, 300, 300},
				{700, 150, 150, 150},
				{700, 0, 150, 150}};
	}

	for (int i = 0; i < 3; i++)
	{
		glViewport(viewport[i][0], viewport[i][1], viewport[i][2], viewport[i][3]);
		setup_view(ctx, flag, i); // XYZ == 0, XY == 1, XZ == 2
		draw_axis(10000000000000);
		glBindVertexArray(VAO);
		glPointSize(3);
		glColor3f(color[0], color[1], color[2]);
		glDrawArrays(GL_POINTS, 0, array_size);
		glBindVertexArray(0);
	}
}

void	render(t_opengl &ctx)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	draw_graph(ctx, POS);
	draw_graph(ctx, COV_P);
	draw_graph(ctx, COV_V);

	glfwSwapBuffers(ctx.window);
}
