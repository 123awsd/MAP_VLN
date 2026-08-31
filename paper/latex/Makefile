.PHONY: all zh en clean check

all: zh en

zh:
	mkdir -p build
	latexmk -xelatex main_zh.tex
	cp build/main_zh.pdf ../main_zh.pdf

en:
	mkdir -p build
	latexmk -xelatex main_en.tex
	cp build/main_en.pdf ../main_en.pdf

check:
	chktex -q -n 8 -n 13 -n 24 main_zh.tex main_en.tex

clean:
	latexmk -C main_zh.tex
	latexmk -C main_en.tex
	rm -rf build
	rm -f ../main_zh.pdf ../main_en.pdf
