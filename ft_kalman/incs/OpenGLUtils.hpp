/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   OpenGLUtils.hpp                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/05/20 09:05:26 by yhwang            #+#    #+#             */
/*   Updated: 2024/05/27 15:14:25 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef OPENGL_UTILS_HPP
# define OPENGL_UTILS_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <functional>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

# define	WINODW_WIDTH	850
# define	WINDOW_HEIGHT	600

# define	POS		0
# define	COV_P		1
# define	COV_V		2

# define	XYZ		0
# define	XY		1
# define	XZ		2

typedef struct s_point3
{
	GLfloat			x;
	GLfloat			y;
	GLfloat			z;
}	t_point3;

typedef struct s_data
{
	float			zoom;
	float			camera_lookat_x;
	float			camera_lookat_y;
	float			camera_lookat_z;

}	t_data;

typedef struct s_opengl
{
	GLFWwindow		*window;
	GLuint			VAO_position;
	GLuint			VAO_covariance_p;
	GLuint			VAO_covariance_v;
	GLuint			VBO_position;
	GLuint			VBO_covariance_p;
	GLuint			VBO_covariance_v;
	std::vector<t_point3>	position;
	std::vector<t_point3>	covariance_p;
	std::vector<t_point3>	covariance_v;
	t_data			data_position;
	t_data			data_covariance_p;
	t_data			data_covariance_v;
}	t_opengl;

bool			init_opengl(t_opengl &ctx);
void			update_position_graph(t_opengl &ctx, const std::vector<double> &pos);
void			update_covariance_graph(t_opengl &ctx, const std::vector<std::vector<double>> &cov);
void			scroll_callback(GLFWwindow *window, double xoffset, double yoffset);
void			key_callback(GLFWwindow *window, int key, int scancode, int action, int mods);
void			setup_view(t_opengl &ctx, int flag, int view_type);
void			draw_axis(double length);
void			draw_graph(t_opengl &ctx, int flag);
void			render(t_opengl &ctx);

#endif
