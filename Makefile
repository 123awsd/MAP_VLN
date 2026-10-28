.PHONY: all pdf check bundle clean

all: pdf

pdf:
	$(MAKE) -C manuscript pdf

check:
	python3 figures/source/project_tools.py check

bundle: check
	python3 figures/source/project_tools.py bundle

clean:
	$(MAKE) -C manuscript clean
