### Recipes ###

# Builder
define Builder
	g++ -o $(1) $(2) -Isrc 
endef

all: output_folder pendulum_pid pendulum_double wheel_simple wheel_kalman
	@echo --- Building all examples

output_folder:
	if not exist _bin (mkdir _bin) 

pendulum_pid: output_folder
	@echo --- Building pendulum_pid
	$(call Builder,pendulum_pid,examples\pendulum_pid.cpp)
	move "pendulum_pid.exe" _bin	

pendulum_double: output_folder
	@echo --- Building pendulum_double
	$(call Builder,pendulum_double,examples\pendulum_double.cpp)
	move "pendulum_double.exe" _bin	

wheel_simple: output_folder
	@echo --- Building wheel_simple
	$(call Builder,wheel_simple,examples\wheel_simple.cpp)
	move "wheel_simple.exe" _bin

wheel_kalman: output_folder
	@echo --- Building wheel_simple
	$(call Builder,wheel_kalman,examples\wheel_kalman.cpp)
	move "wheel_kalman.exe" _bin	

matrix_test: output_folder
	@echo --- Building matrix_test
	$(call Builder,matrix_test,tests\matrix_test.cpp)
	move "matrix_test.exe" _bin

clean: 
	if exist _bin (rmdir /s /q _bin)
	@echo --- Deleted binaries
