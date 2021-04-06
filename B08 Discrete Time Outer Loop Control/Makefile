NAME := outer
LIBS := -lpthread
SRC := receiveUdp.cpp sendUdp.cpp Attitudes.cpp sendFunction.cpp receiveFunction.cpp controllerFunction.cpp main.cpp
HDR := receiveUdp.hpp sendUdp.hpp Attitudes.hpp

# Tool invocations
$(NAME): $(SRC) $(HDR)
	g++ $(LIBS) $(SRC) -o $(NAME)

# Other Targets
clean:
	rm $(NAME)
