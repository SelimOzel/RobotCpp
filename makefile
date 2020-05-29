### Recipes ###

# Builder
define Builder
	g++ -o $(2) $(1) -Isrc 
endef

all: output_folder wheel_simple
	@echo --- Building all examples

output_folder:
	if not exist _bin (mkdir _bin) 

wheel_simple: output_folder
	@echo --- Building wheel_simple
	$(call Builder,examples\wheel_simple.cpp,wheel_simple)
	move "wheel_simple.exe" _bin

clean: 
	if exist _bin (rmdir /s /q _bin)
	@echo --- Deleted binaries
