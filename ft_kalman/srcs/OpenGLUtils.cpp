/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   OpenGLUtils.cpp                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/05/20 16:06:59 by yhwang            #+#    #+#             */
/*   Updated: 2024/05/26 22:47:06 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "../incs/OpenGLUtils.hpp"
#include "../incs/Color.hpp"

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

void	init_data(t_opengl &ctx)
{
	ctx.data_position.zoom = 1.0f;
	ctx.data_position.camera_lookat_x = 14500;
	ctx.data_position.camera_lookat_y = -1000;
	ctx.data_position.camera_lookat_z = -2000;

	ctx.data_covariance_p.zoom = 1.0f;
	ctx.data_covariance_p.camera_lookat_x = 14500;
	ctx.data_covariance_p.camera_lookat_y = -1000;
	ctx.data_covariance_p.camera_lookat_z = -2000;

	ctx.data_covariance_v.zoom = 1.0f;
	ctx.data_covariance_v.camera_lookat_x = 14500;
	ctx.data_covariance_v.camera_lookat_y = -1000;
	ctx.data_covariance_v.camera_lookat_z = -2000;
}

bool	init_opengl(t_opengl &ctx)
{
	init_data(ctx);
	if (!glfwInit())
	{
		std::cerr << RED << "error: failed to initialize GLFW" << BLACK << std::endl;
		return (false);
	}

	ctx.window = glfwCreateWindow(WINODW_WIDTH, WINDOW_HEIGHT, "Kalman Filter", NULL, NULL);
	if (!ctx.window)
	{
		glfwTerminate();
		std::cerr << RED << "error: failed to create GLFW window" << BLACK << std::endl;
		return (false);
	}

	glfwMakeContextCurrent(ctx.window);
	glfwSetWindowUserPointer(ctx.window, &ctx);
	glfwSetScrollCallback(ctx.window, scroll_callback);
	glfwSetKeyCallback(ctx.window, key_callback);

	if (glewInit() != GLEW_OK)
	{
		std::cerr << RED << "error: failed to initialize GLFW" << BLACK << std::endl;
		return (false);
	}

	glGenVertexArrays(1, &ctx.VAO_position);
	glBindVertexArray(ctx.VAO_position);
	glGenBuffers(1, &ctx.VBO_position);
	glBindBuffer(GL_ARRAY_BUFFER, ctx.VBO_position);
	glBufferData(GL_ARRAY_BUFFER, sizeof(t_point3) * ctx.position.size(), nullptr, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(t_point3), (void *)0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glGenVertexArrays(1, &ctx.VAO_covariance_p);
	glBindVertexArray(ctx.VAO_covariance_p);
	glGenBuffers(1, &ctx.VBO_covariance_p);
	glBindBuffer(GL_ARRAY_BUFFER, ctx.VBO_covariance_p);
	glBufferData(GL_ARRAY_BUFFER, sizeof(t_point3) * ctx.covariance_p.size(), nullptr, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(t_point3), (void *)0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glGenVertexArrays(1, &ctx.VAO_covariance_v);
	glBindVertexArray(ctx.VAO_covariance_v);
	glGenBuffers(1, &ctx.VBO_covariance_v);
	glBindBuffer(GL_ARRAY_BUFFER, ctx.VBO_covariance_v);
	glBufferData(GL_ARRAY_BUFFER, sizeof(t_point3) * ctx.covariance_v.size(), nullptr, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(t_point3), (void *)0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // white background
	return (true);
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
	else if (flag == COV_V)
	{
		zoom = ctx.data_covariance_v.zoom;
		camera_lookat_x = ctx.data_covariance_v.camera_lookat_x;
		camera_lookat_y = ctx.data_covariance_v.camera_lookat_y;
		camera_lookat_z = ctx.data_covariance_v.camera_lookat_z;
	}

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(30 * zoom, 1 / 1, 1, 10000000000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	if (view_type == 0) // 3d view
		gluLookAt(45000 / zoom, -45000 / zoom, 45000 / zoom, // camera position
			camera_lookat_x / zoom, camera_lookat_y / zoom, camera_lookat_z / zoom, // looks at
			0, 0, 1); // Carmera vector
	if (view_type == 1)
		gluLookAt(0, 0, 90000 / zoom,
			14000, 0, 0,
			0, 1, 0);
	if (view_type == 2)
		gluLookAt(0, -90000 / zoom, 0,
			14000, 0, 0,
			0, 0, 1);
	(void)ctx;
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

void	render(t_opengl &ctx)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glViewport(0, 200, 400, 400);
	setup_view(ctx, POS, XYZ);
	draw_axis(10000000000);
	glBindVertexArray(ctx.VAO_position);
	glPointSize(3);
	glColor3f(1, 0, 1);
	glDrawArrays(GL_POINTS, 0, ctx.position.size());
	glBindVertexArray(0);

	glViewport(0, 0, 200, 200);
	setup_view(ctx, POS, XY);
	draw_axis(10000000000);
	glBindVertexArray(ctx.VAO_position);
	glPointSize(3);
	glColor3f(1, 0, 1);
	glDrawArrays(GL_POINTS, 0, ctx.position.size());
	glBindVertexArray(0);

	glViewport(200, 0, 200, 200);
	setup_view(ctx, POS, XZ);
	draw_axis(10000000000);
	glBindVertexArray(ctx.VAO_position);
	glPointSize(3);
	glColor3f(1, 0, 1);
	glDrawArrays(GL_POINTS, 0, ctx.position.size());
	glBindVertexArray(0);

	glViewport(400, 300, 300, 300);
	setup_view(ctx, COV_P, XYZ);
	draw_axis(100000);
	glBindVertexArray(ctx.VAO_covariance_p);
	glPointSize(3);
	glColor3f(0, 0, 0);
	glDrawArrays(GL_POINTS, 0, ctx.covariance_p.size());
	glBindVertexArray(0);

	glViewport(700, 450, 150, 150);
	setup_view(ctx, COV_P, XY);
	draw_axis(100000);
	glBindVertexArray(ctx.VAO_covariance_p);
	glPointSize(3);
	glColor3f(0, 0, 0);
	glDrawArrays(GL_POINTS, 0, ctx.covariance_p.size());
	glBindVertexArray(0);

	glViewport(700, 300, 150, 150);
	setup_view(ctx, COV_P, XZ);
	draw_axis(100000);
	glBindVertexArray(ctx.VAO_covariance_p);
	glPointSize(3);
	glColor3f(0, 0, 0);
	glDrawArrays(GL_POINTS, 0, ctx.covariance_p.size());
	glBindVertexArray(0);

	glViewport(400, 0, 300, 300);
	setup_view(ctx, COV_V, XYZ);
	draw_axis(100000);
	glBindVertexArray(ctx.VAO_covariance_v);
	glPointSize(3);
	glColor3f(0.5, 0.5, 0.5);
	glDrawArrays(GL_POINTS, 0, ctx.covariance_v.size());
	glBindVertexArray(0);

	glViewport(700, 150, 150, 150);
	setup_view(ctx, COV_V, XY);
	draw_axis(100000);
	glBindVertexArray(ctx.VAO_covariance_v);
	glPointSize(3);
	glColor3f(0.5, 0.5, 0.5);
	glDrawArrays(GL_POINTS, 0, ctx.covariance_v.size());
	glBindVertexArray(0);

	glViewport(700, 0, 150, 150);
	setup_view(ctx, COV_V, XZ);
	draw_axis(100000);
	glBindVertexArray(ctx.VAO_covariance_v);
	glPointSize(3);
	glColor3f(0.5, 0.5, 0.5);
	glDrawArrays(GL_POINTS, 0, ctx.covariance_v.size());
	glBindVertexArray(0);

	glfwSwapBuffers(ctx.window);
}
