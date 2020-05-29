### Recipes ###

# Builder
define Builder
	g++ $(1) 
	mv $(2) bin
	mv $(3) obj
	@echo --- Completed $(1) 
endef

all: output_folders wheel_simple
	@echo --- Building all examples

output_folders:
	if not exist examples\_bin (mkdir examples\_bin examples\_data) 

wheel_simple: output_folders
	@echo --- Building wheel_simple

clean: 
	if exist examples\_bin (rmdir /s /q examples\_bin examples\_data)
	@echo --- Deleted binaries and output data

