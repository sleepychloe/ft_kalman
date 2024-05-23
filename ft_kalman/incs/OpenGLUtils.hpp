/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   OpenGLUtils.hpp                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/05/20 09:05:26 by yhwang            #+#    #+#             */
/*   Updated: 2024/05/24 00:55:10 by yhwang           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef OPENGL_UTILS_HPP
# define OPENGL_UTILS_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

# define	WINODW_WIDTH	400
# define	WINDOW_HEIGHT	600

# define	POS_TRUE	0
# define	POS_ESTIMATE	1

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
	GLuint			VAO;
	GLuint			VBO;
	std::vector<t_point3>	pos_estimate;
	std::vector<t_point3>	pos_true;
	t_data			data;
}	t_opengl;

void			scroll_callback(GLFWwindow *window, double xoffset, double yoffset);
void			key_callback(GLFWwindow *window, int key, int scancode, int action, int mods);
void			init_data(t_opengl &ctx);
bool			init_opengl(t_opengl &ctx);
void			setup_view(t_opengl &ctx, int view_type);
void			update_graph(t_opengl &ctx, const std::vector<double> &pos);
void			draw_axis(double length);
void			render(t_opengl &ctx);

#endif
