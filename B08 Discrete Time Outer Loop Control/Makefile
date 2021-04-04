NAME := outer
LIBS := -lpthread
SRC := receiveUdp.cpp sendUdp.cpp
HDR := receiveUdp.hpp sendUdp.hpp

# Tool invocations
$(NAME): $(SRC) $(HDR)
	g++ $(LIBS) $(SRC) -o $(NAME)

# Other Targets
clean:
	rm $(NAME)
