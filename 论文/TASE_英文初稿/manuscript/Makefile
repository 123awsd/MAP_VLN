.PHONY: all pdf clean

all: pdf

pdf:
	latexmk -xelatex -outdir=build root.tex
	cp build/root.pdf root.pdf

clean:
	latexmk -c -outdir=build root.tex
