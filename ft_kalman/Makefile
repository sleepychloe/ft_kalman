# **************************************************************************** #
#                                                                              #
#                                                         :::      ::::::::    #
#    Makefile                                           :+:      :+:    :+:    #
#                                                     +:+ +:+         +:+      #
#    By: yhwang <yhwang@student.42.fr>              +#+  +:+       +#+         #
#                                                 +#+#+#+#+#+   +#+            #
#    Created: 2024/04/23 23:34:22 by yhwang            #+#    #+#              #
#    Updated: 2024/04/25 01:18:19 by yhwang           ###   ########.fr        #
#                                                                              #
# **************************************************************************** #

NAME = ft_kalman
CPP = c++
CFLAGS = -Wall -Wextra -Werror -std=c++17 -g3
RM = rm -f

SRC_DIR = ./srcs/

FILES = $(SRC_DIR)main \
		$(SRC_DIR)Parse \
		$(SRC_DIR)Utils \
		$(SRC_DIR)KalmanFilter

INCS_DIR = ./incs/

SRCS = $(addsuffix .cpp, $(FILES))
OBJS = $(addsuffix .o, $(FILES))

%.o: %.cpp $(INCS_DIR)
	$(CPP) $(CFLAGS) -c $< -o $@ -I$(INCS_DIR)

all: $(NAME)

$(NAME): $(OBJS)
	$(CPP) $(CFLAGS) $^ -o $@ -I$(INCS_DIR)

clean:
	$(RM) $(OBJS)

fclean: clean
	$(RM) $(NAME)

re: fclean all

.PHONY: all clean fclean re
