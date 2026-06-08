.PHONY: build laptop chair backpack clean

build:
	colcon build --symlink-install

laptop:
	./run.sh laptop

chair:
	./run.sh chair

backpack:
	./run.sh backpack

clean:
	rm -rf build install log
