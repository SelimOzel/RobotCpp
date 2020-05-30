### Recipes ###

# Builder
define Builder
	g++ -o $(1) $(2) -Isrc 
endef

all: output_folder wheel_simple wheel_kalman
	@echo --- Building all examples

output_folder:
	if not exist _bin (mkdir _bin) 

wheel_simple: output_folder
	@echo --- Building wheel_simple
	$(call Builder,wheel_simple,examples\wheel_simple.cpp)
	move "wheel_simple.exe" _bin

wheel_kalman: output_folder
	@echo --- Building wheel_simple
	$(call Builder,wheel_kalman,examples\wheel_kalman.cpp)
	move "wheel_kalman.exe" _bin	

clean: 
	if exist _bin (rmdir /s /q _bin)
	@echo --- Deleted binaries
