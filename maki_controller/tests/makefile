FLAGS = -g -O0 -Wall

test: controller_test.cpp
	g++ -o $@ $(FLAGS) $^

clean:
	rm test
	
remake: clean test
