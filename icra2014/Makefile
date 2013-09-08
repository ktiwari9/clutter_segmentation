
#outputs $(PROJFILE).{dvi,pdf,ps,...}
PROJNAME = icra2014

#do not include .tex
SOURCEFILE = main

#other files $(SOURCEFILE).tex includes, eg .bib files, etc.
#DO include .tex, .bib, ...
INCLUDES = texFiles/ICRA2014_Experiments.tex texFiles/ICRA2014_Conclusion.tex texFiles/ICRA2014_HypothesisTesting.tex texFiles/ICRA2014_Introduction.tex texFiles/ICRA2014_ObservationModel.tex texFiles/ICRA2014_ProblemFormulation.tex texFiles/ICRA2014_RelatedWork.tex texFiles/ICRA2014_ObjectDetection.tex
#ENV      = env TEXINPUTS=".:styles/:" BSTINPUTS=".:styles/:"
ENV      =
BIBTEX   = $(ENV) bibtex
LATEX    = $(ENV) latex
DVIPS    = dvips
PDFLATEX = $(ENV) pdflatex
EPSTOPDF = epstopdf

default: pdf

.PHONY: default all dvi ps pdf clean clobber

all: dvi ps pdf

view: pdf
	acroread $(PROJNAME).pdf

dvi: $(PROJNAME).dvi
ps:  $(PROJNAME).ps
pdf: $(PROJNAME).pdf

$(PROJNAME).dvi: $(SOURCEFILE).tex $(INCLUDES)
	$(LATEX) $(SOURCEFILE).tex
	$(BIBTEX) $(SOURCEFILE).aux
	$(LATEX) $(SOURCEFILE).tex
	$(LATEX) $(SOURCEFILE).tex
	$(LATEX) $(SOURCEFILE).tex
	mv -f $(SOURCEFILE).dvi $(PROJNAME).dvi

$(PROJNAME).ps: $(PROJNAME).dvi
	$(DVIPS) $(PROJNAME).dvi -o $(PROJNAME).ps

$(PROJNAME).pdf: $(SOURCEFILE).tex $(INCLUDES)
	$(PDFLATEX) $(SOURCEFILE).tex
	$(BIBTEX) $(SOURCEFILE).aux
	$(PDFLATEX) $(SOURCEFILE).tex
	$(PDFLATEX) $(SOURCEFILE).tex
	$(PDFLATEX) $(SOURCEFILE).tex
	mv -f $(SOURCEFILE).pdf $(PROJNAME).pdf

clean:
	rm -f $(PROJNAME).aux $(PROJNAME).log $(PROJNAME).nav $(PROJNAME).out $(PROJNAME).snm $(PROJNAME).toc $(PROJNAME).bbl $(PROJNAME).blg $(PROJNAME)_out.aux $(PROJNAME)_out.log $(PROJNAME)_out.nav $(PROJNAME)_out.snm $(PROJNAME)_out.toc $(PROJNAME)_out.tex $(SOURCEFILE).dvi $(SOURCEFILE).pdf $(SOURCEFILE).aux $(SOURCEFILE).log $(SOURCEFILE).nav $(SOURCEFILE).out $(SOURCEFILE).snm $(SOURCEFILE).toc $(SOURCEFILE).bbl $(SOURCEFILE).blg

clobber: clean
	rm -f $(PROJNAME).dvi $(PROJNAME).ps $(PROJNAME).pdf

