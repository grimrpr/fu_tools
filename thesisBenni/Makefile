TARGET?=main-text

default: build

clean:
	rm -f *.acn *.acr *.alg *.aux *.bbl *.blg *.dvi *.glg *.glo *.gls *.ist *.lof *.log *.lol *.lot *.out *.pdf *.toc
build: 
	pdflatex $(TARGET).tex
	makeindex -s $(TARGET).ist -t $(TARGET).glg -o $(TARGET).gls $(TARGET).glo
	makeindex -s $(TARGET).ist -t $(TARGET).alg -o $(TARGET).acr $(TARGET).acn
	pdflatex $(TARGET).tex
